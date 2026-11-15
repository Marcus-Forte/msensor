#include "msensor/recorder/bus_recorder.hh"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <format>
#include <stdexcept>
#include <utility>

#include "msensor/file/file.hh"
#include "msensor/log/log.hh"
#include "msensor/recorder/scan_recorder.hh"

namespace msensor {
namespace {

using Clock = std::chrono::steady_clock;
constexpr auto kLogInterval = std::chrono::seconds(1);

/// One recorded stream: its subscription and its counters. `written` and
/// `failed` are touched by the writer thread; `warn` only by the topic's
/// dispatcher thread (through the wake-up hook).
template <class T> struct StreamState {
  const char *name = "";
  std::unique_ptr<Subscription<T>> sub;
  std::atomic<size_t> written{0};
  std::atomic<size_t> failed{0};
  size_t dropped_final = 0; // read from the subscription at stop

  // Dispatcher thread only.
  size_t warned_dropped = 0;
  Clock::time_point last_warn{};
  bool warned_once = false;

  // Writer thread only.
  Clock::time_point last_error{};
  bool errored_once = false;

  size_t dropped() const { return sub ? sub->dropped() : dropped_final; }

  /// Warns when samples were dropped since the last warning, at most once per
  /// interval.
  void warnOnDrop(const Subscription<T> &watched, size_t capacity) {
    const size_t dropped_now = watched.dropped();
    if (dropped_now == warned_dropped) {
      return;
    }
    const auto now = Clock::now();
    if (warned_once && now - last_warn < kLogInterval) {
      return;
    }
    log::warn("Recorder: dropped {} {} so far ({} since the last warning): "
              "the disk writer is not keeping up with the {}-entry queue.",
              dropped_now, name, dropped_now - warned_dropped, capacity);
    warned_dropped = dropped_now;
    last_warn = now;
    warned_once = true;
  }
};

} // namespace

struct BusRecorder::Impl {
  explicit Impl(std::shared_ptr<IFile> file) : recorder(std::move(file)) {}

  ScanRecorder recorder;
  StreamState<Scan3DI> lidar;
  StreamState<IMUData> imu;
};

BusRecorder::BusRecorder(std::shared_ptr<SensorBus> bus,
                         BusRecorderOptions options,
                         std::shared_ptr<IFile> file)
    : bus_(std::move(bus)), options_(options),
      impl_(std::make_unique<Impl>(file ? std::move(file)
                                        : std::make_shared<File>())) {
  options_.scan_queue = std::max<size_t>(1, options_.scan_queue);
  options_.imu_queue = std::max<size_t>(1, options_.imu_queue);
  impl_->lidar.name = "LiDAR scans";
  impl_->imu.name = "IMU samples";
}

BusRecorder::~BusRecorder() {
  try {
    stop();
  } catch (const std::exception &e) {
    log::error("Recorder: stop failed: {}", e.what());
  } catch (...) {
    log::error("Recorder: stop failed.");
  }
}

void BusRecorder::wake() {
  {
    std::lock_guard lock(wake_m_);
    wake_ = true;
  }
  wake_cv_.notify_one();
}

void BusRecorder::start(const std::string &filename) {
  std::lock_guard control(control_m_);
  if (started_) {
    throw std::logic_error("BusRecorder is already started.");
  }
  if (!bus_ || (!bus_->lidar() && !bus_->imu())) {
    throw std::logic_error(
        "BusRecorder needs a bus with a LiDAR or an IMU topic.");
  }

  // Open the file first: a failure leaves nothing attached to the drivers.
  if (filename.empty()) {
    impl_->recorder.start();
  } else {
    impl_->recorder.start(filename);
  }
  filename_ = impl_->recorder.getFilename();

  try {
    // Subscribe, then install the hooks; queued samples fire the hook too.
    if (Topic<IMUData> *topic = bus_->imu()) {
      impl_->imu.sub = topic->subscribe(options_.imu_queue);
      // The hook gets the raw pointer: `sub` itself is reset by stop() while
      // a hook may still be running (the reset waits for it).
      impl_->imu.sub->setWakeHook([this, watched = impl_->imu.sub.get()] {
        impl_->imu.warnOnDrop(*watched, options_.imu_queue);
        wake();
      });
    }
    if (Topic<Scan3DI> *topic = bus_->lidar()) {
      impl_->lidar.sub = topic->subscribe(options_.scan_queue);
      impl_->lidar.sub->setWakeHook([this, watched = impl_->lidar.sub.get()] {
        impl_->lidar.warnOnDrop(*watched, options_.scan_queue);
        wake();
      });
    }
    writer_ = std::jthread([this](std::stop_token st) { run(st); });
  } catch (...) {
    impl_->imu.sub.reset();
    impl_->lidar.sub.reset();
    impl_->recorder.stop();
    throw;
  }
  started_ = true;
  log::info("Recorder: recording to {} (LiDAR queue {}, IMU queue {}).",
            filename_, impl_->lidar.sub ? options_.scan_queue : 0,
            impl_->imu.sub ? options_.imu_queue : 0);
}

void BusRecorder::run(std::stop_token st) {
  for (;;) {
    {
      std::unique_lock lock(wake_m_);
      if (!wake_cv_.wait(lock, st, [this] { return wake_; })) {
        break; // stop requested
      }
      wake_ = false;
    }
    drain();
  }
  drain(); // what is queued at stop
}

void BusRecorder::drain() {
  // Take the sizes first: the scan count before the IMU count, so every IMU
  // sample published before a scan that is in this pass is in this pass too,
  // and is written before it. A producer that outpaces the disk cannot keep
  // one pass going, because the counts are fixed here.
  const size_t scans = impl_->lidar.sub ? impl_->lidar.sub->size() : 0;
  const size_t imus = impl_->imu.sub ? impl_->imu.sub->size() : 0;

  const auto pass = [this](auto &stream, size_t count) {
    for (; count > 0; --count) {
      const auto sample = stream.sub->tryPop();
      if (!sample) {
        break;
      }
      try {
        impl_->recorder.record(*sample);
        stream.written.fetch_add(1, std::memory_order_relaxed);
      } catch (const std::exception &e) {
        stream.failed.fetch_add(1, std::memory_order_relaxed);
        const auto now = Clock::now();
        if (!stream.errored_once || now - stream.last_error >= kLogInterval) {
          log::error("Recorder: cannot write {}: {}", stream.name, e.what());
          stream.last_error = now;
          stream.errored_once = true;
        }
      }
    }
  };
  pass(impl_->imu, imus);
  pass(impl_->lidar, scans);
}

BusRecorderStats BusRecorder::stop() {
  std::lock_guard control(control_m_);
  if (!started_ || stopped_) {
    return final_stats_;
  }
  stopped_ = true;

  writer_.request_stop();
  if (writer_.joinable()) {
    writer_.join(); // drains once more before it returns
  }
  // Detach from the topics (a barrier for the hooks), keeping the drop counts.
  if (impl_->imu.sub) {
    impl_->imu.dropped_final = impl_->imu.sub->dropped();
  }
  if (impl_->lidar.sub) {
    impl_->lidar.dropped_final = impl_->lidar.sub->dropped();
  }
  final_stats_ = {};
  final_stats_.lidar = {impl_->lidar.sub != nullptr,
                        impl_->lidar.written.load(), impl_->lidar.dropped_final,
                        impl_->lidar.failed.load()};
  final_stats_.imu = {impl_->imu.sub != nullptr, impl_->imu.written.load(),
                      impl_->imu.dropped_final, impl_->imu.failed.load()};
  impl_->imu.sub.reset();
  impl_->lidar.sub.reset();

  try {
    impl_->recorder.stop();
  } catch (const std::exception &e) {
    log::error("Recorder: closing {} failed: {}", filename_, e.what());
  }

  const auto line = [](const char *name, const BusRecorderStreamStats &s) {
    if (s.recorded) {
      log::info("Recorder: {}: {} written, {} dropped{}.", name, s.written,
                s.dropped,
                s.failed ? std::format(", {} failed to write", s.failed)
                         : std::string());
    }
  };
  log::info("Recorder: stopped, {} closed.", filename_);
  line("LiDAR scans", final_stats_.lidar);
  line("IMU samples", final_stats_.imu);
  return final_stats_;
}

BusRecorderStats BusRecorder::stats() const {
  std::lock_guard control(control_m_);
  if (!started_ || stopped_) {
    return final_stats_;
  }
  BusRecorderStats out;
  out.lidar = {impl_->lidar.sub != nullptr, impl_->lidar.written.load(),
               impl_->lidar.dropped(), impl_->lidar.failed.load()};
  out.imu = {impl_->imu.sub != nullptr, impl_->imu.written.load(),
             impl_->imu.dropped(), impl_->imu.failed.load()};
  return out;
}

bool BusRecorder::running() const {
  std::lock_guard control(control_m_);
  return started_ && !stopped_;
}

std::string BusRecorder::filename() const {
  std::lock_guard control(control_m_);
  return filename_;
}

} // namespace msensor
