// Tests of BusRecorder: recording a SensorBus to a .pbscan file.

#include "msensor/bus/sensor_bus.hh"
#include "msensor/config/config.hh"
#include "msensor/file/file.hh"
#include "msensor/interface/CallbackSlot.hh"
#include "msensor/log/log.hh"
#include "msensor/recorder/bus_recorder.hh"
#include "msensor/recorder/scan_player.hh"
#include "recording.pb.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

namespace {

/// Emits exactly the samples a test pushes, like a driver's sampling thread.
class FeedSensor : public msensor::ILidar, public msensor::IImu {
public:
  void init() override {}
  void startSampling() override {}
  void stopSampling() override {}
  void setScanCallback(ScanCallback callback) override {
    scans_.setCallback(std::move(callback));
  }
  void setImuCallback(ImuCallback callback) override {
    imus_.setCallback(std::move(callback));
  }
  bool hasScanConsumer() const { return scans_.hasCallback(); }
  bool hasImuConsumer() const { return imus_.hasCallback(); }
  void push(msensor::Scan3DI scan) { scans_.emit(std::move(scan)); }
  void push(msensor::IMUData imu) { imus_.emit(std::move(imu)); }

private:
  msensor::CallbackSlot<msensor::Scan3DI> scans_;
  msensor::CallbackSlot<msensor::IMUData> imus_;
};

msensor::Scan3DI makeScan(uint32_t seq) {
  msensor::Scan3DI scan;
  scan.header = {1'000'000'000ULL + seq, seq};
  for (uint32_t i = 0; i < 3 + seq % 5; ++i) {
    const float f = static_cast<float>(seq) + 0.25F * static_cast<float>(i);
    scan.points.push_back({f, -f, 2 * f, static_cast<float>(i + 1)});
  }
  return scan;
}

msensor::IMUData makeImu(uint32_t seq) {
  const auto f = static_cast<float>(seq);
  return {{2'000'000'000ULL + seq, seq}, f, f + 1, f + 2, -f, -f - 1, -f - 2};
}

/// What a recording holds, in file order.
struct Contents {
  struct Entry {
    bool is_scan;
    uint32_t seq;
    size_t points; // scans only
  };
  std::vector<Entry> entries;
  std::vector<uint32_t> scans, imus;
};

Contents readBack(const std::string &file) {
  Contents out;
  msensor::ScanPlayer player(file);
  while (player.next()) {
    const auto &entry = player.getLastEntry();
    if (entry.has_scan()) {
      const uint32_t seq = entry.scan().header().sequence_number();
      out.entries.push_back(
          {true, seq, static_cast<size_t>(entry.scan().x_size())});
      out.scans.push_back(seq);
    } else if (entry.has_imu()) {
      const uint32_t seq = entry.imu().header().sequence_number();
      out.entries.push_back({false, seq, 0});
      out.imus.push_back(seq);
    }
  }
  return out;
}

class TempFile {
public:
  TempFile()
      : path_(std::filesystem::temp_directory_path() /
              ("msensor_bus_recorder_" + std::to_string(::getpid()) + "_" +
               std::to_string(counter_++) + ".pbscan")) {}
  ~TempFile() {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }
  std::string str() const { return path_.string(); }

private:
  static inline std::atomic<int> counter_{0};
  std::filesystem::path path_;
};

/// A real file whose write() blocks while the gate is closed, to hold the
/// writer thread in the middle of an entry.
class GatedFile : public msensor::IFile {
public:
  void open(const std::string &filename) override { inner_.open(filename); }
  void write(const char *data, size_t size) override {
    {
      std::unique_lock lock(m_);
      if (!open_) {
        ++blocked_;
        cv_.wait(lock, [this] { return open_; });
        --blocked_;
      }
    }
    inner_.write(data, size);
  }
  void flush() override { inner_.flush(); }
  void close() override { inner_.close(); }

  void closeGate() {
    std::lock_guard lock(m_);
    open_ = false;
  }
  void openGate() {
    {
      std::lock_guard lock(m_);
      open_ = true;
    }
    cv_.notify_all();
  }
  bool writerBlocked() {
    std::lock_guard lock(m_);
    return blocked_ > 0;
  }

private:
  msensor::File inner_;
  std::mutex m_;
  std::condition_variable cv_;
  bool open_ = true;
  int blocked_ = 0;
};

template <class Pred>
bool waitFor(Pred pred, std::chrono::milliseconds limit = 10s) {
  const auto end = std::chrono::steady_clock::now() + limit;
  while (!pred()) {
    if (std::chrono::steady_clock::now() > end) {
      return false;
    }
    std::this_thread::sleep_for(200us);
  }
  return true;
}

/// Collects log messages of one level for the test's lifetime.
class LogCapture {
public:
  explicit LogCapture(msensor::log::Level level) : level_(level) {
    msensor::log::setSink([this](msensor::log::Level l, std::string_view m) {
      if (l == level_) {
        std::lock_guard lock(m_);
        messages_.emplace_back(m);
      }
    });
  }
  ~LogCapture() { msensor::log::setSink({}); }
  size_t count() const {
    std::lock_guard lock(m_);
    return messages_.size();
  }
  std::vector<std::string> messages() const {
    std::lock_guard lock(m_);
    return messages_;
  }

private:
  msensor::log::Level level_;
  mutable std::mutex m_;
  std::vector<std::string> messages_;
};

/// A bus over the feed's LiDAR and/or IMU (field by field: partial
/// designated initializers warn).
std::shared_ptr<msensor::SensorBus> busOf(const std::shared_ptr<FeedSensor> &s,
                                          bool lidar = true, bool imu = true) {
  msensor::Sensors sensors;
  if (lidar) {
    sensors.lidar = s;
  }
  if (imu) {
    sensors.imu = s;
  }
  return std::make_shared<msensor::SensorBus>(std::move(sensors));
}

} // namespace

// Invariant 7: a known sample sequence is read back complete and in order by
// ScanPlayer, with the values intact.
TEST(BusRecorder, RecordingOfAKnownSequenceIsReadBackCompleteAndInOrder) {
  constexpr uint32_t kScans = 200;
  constexpr uint32_t kImuPerScan = 4;
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed);
  TempFile file;

  msensor::BusRecorder recorder(bus);
  recorder.start(file.str());
  EXPECT_TRUE(recorder.running());
  EXPECT_TRUE(feed->hasScanConsumer());
  EXPECT_TRUE(feed->hasImuConsumer());

  std::thread producer([&] {
    for (uint32_t k = 0; k < kScans; ++k) {
      for (uint32_t j = 0; j < kImuPerScan; ++j) {
        feed->push(makeImu(k * kImuPerScan + j));
      }
      feed->push(makeScan(k));
    }
  });
  producer.join();
  const auto stats = recorder.stop();

  EXPECT_FALSE(recorder.running());
  EXPECT_EQ(stats.lidar.written, kScans);
  EXPECT_EQ(stats.imu.written, kScans * kImuPerScan);
  EXPECT_EQ(stats.lidar.dropped, 0U);
  EXPECT_EQ(stats.imu.dropped, 0U);
  EXPECT_EQ(stats.lidar.failed + stats.imu.failed, 0U);
  // Stopping detaches from the drivers.
  EXPECT_FALSE(feed->hasScanConsumer());
  EXPECT_FALSE(feed->hasImuConsumer());

  const Contents got = readBack(file.str());
  ASSERT_EQ(got.scans.size(), kScans);
  ASSERT_EQ(got.imus.size(), kScans * kImuPerScan);
  for (uint32_t i = 0; i < got.scans.size(); ++i) {
    ASSERT_EQ(got.scans[i], i);
  }
  for (uint32_t i = 0; i < got.imus.size(); ++i) {
    ASSERT_EQ(got.imus[i], i);
  }

  // Values, not just numbers.
  msensor::ScanPlayer player(file.str());
  bool checked_scan = false;
  bool checked_imu = false;
  while (player.next()) {
    const auto &entry = player.getLastEntry();
    if (entry.has_scan() && entry.scan().header().sequence_number() == 7) {
      const auto want = makeScan(7);
      ASSERT_EQ(static_cast<size_t>(entry.scan().x_size()), want.points.size());
      EXPECT_EQ(entry.scan().header().timestamp(), want.header.timestamp);
      for (int i = 0; i < entry.scan().x_size(); ++i) {
        EXPECT_EQ(entry.scan().x(i), want.points[static_cast<size_t>(i)].x);
        EXPECT_EQ(entry.scan().intensity(i),
                  want.points[static_cast<size_t>(i)].intensity);
      }
      checked_scan = true;
    }
    if (entry.has_imu() && entry.imu().header().sequence_number() == 33) {
      const auto want = makeImu(33);
      EXPECT_EQ(entry.imu().header().timestamp(), want.header.timestamp);
      EXPECT_EQ(entry.imu().ax(), want.ax);
      EXPECT_EQ(entry.imu().gz(), want.gz);
      checked_imu = true;
    }
  }
  EXPECT_TRUE(checked_scan);
  EXPECT_TRUE(checked_imu);
}

// The IMU samples published before a scan are in the file before it.
TEST(BusRecorder, ImuSamplesOfAScanPrecedeItInTheFile) {
  constexpr uint32_t kScans = 300;
  constexpr uint32_t kImuPerScan = 3;
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed);
  TempFile file;
  msensor::BusRecorder recorder(bus);
  recorder.start(file.str());
  for (uint32_t k = 0; k < kScans; ++k) {
    for (uint32_t j = 0; j < kImuPerScan; ++j) {
      feed->push(makeImu(k * kImuPerScan + j));
    }
    feed->push(makeScan(k));
    if (k % 16 == 0) {
      std::this_thread::sleep_for(1ms); // vary the wake-up boundaries
    }
  }
  recorder.stop();

  const Contents got = readBack(file.str());
  size_t imus_seen = 0;
  size_t scans_seen = 0;
  for (const auto &e : got.entries) {
    if (e.is_scan) {
      ++scans_seen;
      ASSERT_GE(imus_seen, scans_seen * kImuPerScan)
          << "scan " << e.seq << " precedes its IMU samples";
    } else {
      ++imus_seen;
    }
  }
  EXPECT_EQ(scans_seen, kScans);
  EXPECT_EQ(imus_seen, kScans * kImuPerScan);
}

TEST(BusRecorder, RecordsAnImuOnlyBus) {
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed, false, true);
  TempFile file;
  msensor::BusRecorder recorder(bus);
  recorder.start(file.str());
  EXPECT_TRUE(feed->hasImuConsumer());
  for (uint32_t i = 0; i < 50; ++i) {
    feed->push(makeImu(i));
  }
  const auto stats = recorder.stop();
  EXPECT_FALSE(stats.lidar.recorded);
  EXPECT_TRUE(stats.imu.recorded);
  EXPECT_EQ(stats.imu.written, 50U);
  const Contents got = readBack(file.str());
  EXPECT_EQ(got.imus.size(), 50U);
  EXPECT_TRUE(got.scans.empty());
}

TEST(BusRecorder, RecordsALidarOnlyBus) {
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed, true, false);
  TempFile file;
  msensor::BusRecorder recorder(bus);
  recorder.start(file.str());
  EXPECT_TRUE(feed->hasScanConsumer());
  EXPECT_FALSE(feed->hasImuConsumer());
  for (uint32_t i = 0; i < 20; ++i) {
    feed->push(makeScan(i));
  }
  const auto stats = recorder.stop();
  EXPECT_TRUE(stats.lidar.recorded);
  EXPECT_FALSE(stats.imu.recorded);
  EXPECT_EQ(stats.lidar.written, 20U);
  const Contents got = readBack(file.str());
  EXPECT_EQ(got.scans.size(), 20U);
  EXPECT_TRUE(got.imus.empty());
}

TEST(BusRecorder, ABusWithoutLidarAndImuCannotBeRecorded) {
  auto bus = std::make_shared<msensor::SensorBus>(msensor::Sensors{});
  TempFile file;
  msensor::BusRecorder recorder(bus);
  EXPECT_THROW(recorder.start(file.str()), std::logic_error);
  EXPECT_FALSE(recorder.running());
  msensor::BusRecorder without_bus(nullptr);
  EXPECT_THROW(without_bus.start(file.str()), std::logic_error);
}

TEST(BusRecorder, AFileThatCannotBeOpenedLeavesTheDriversUntouched) {
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed);
  msensor::BusRecorder recorder(bus);
  EXPECT_THROW(recorder.start("/nonexistent-directory/x.pbscan"),
               std::runtime_error);
  EXPECT_FALSE(recorder.running());
  EXPECT_FALSE(feed->hasScanConsumer());
  EXPECT_FALSE(feed->hasImuConsumer());
}

TEST(BusRecorder, StopIsIdempotentAndARecorderRecordsOnce) {
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed);
  TempFile file;
  msensor::BusRecorder recorder(bus);
  EXPECT_NO_THROW(recorder.stop()); // before start
  recorder.start(file.str());
  EXPECT_THROW(recorder.start(file.str()), std::logic_error);
  feed->push(makeScan(0));
  const auto first = recorder.stop();
  const auto second = recorder.stop();
  EXPECT_EQ(first.lidar.written, 1U);
  EXPECT_EQ(second.lidar.written, 1U);
  EXPECT_EQ(recorder.stats().lidar.written, 1U);
  EXPECT_THROW(recorder.start(file.str()), std::logic_error);
}

TEST(BusRecorder, DestructorStopsAndWritesWhatIsQueued) {
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed);
  TempFile file;
  {
    msensor::BusRecorder recorder(bus);
    recorder.start(file.str());
    for (uint32_t i = 0; i < 30; ++i) {
      feed->push(makeScan(i));
    }
  }
  EXPECT_FALSE(feed->hasScanConsumer());
  EXPECT_EQ(readBack(file.str()).scans.size(), 30U);
}

TEST(BusRecorder, ADefaultFileNameIsGenerated) {
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed);
  const auto dir = std::filesystem::temp_directory_path() /
                   ("msensor_bus_recorder_cwd_" + std::to_string(::getpid()));
  std::filesystem::create_directory(dir);
  const auto old_cwd = std::filesystem::current_path();
  std::filesystem::current_path(dir);
  std::string name;
  {
    msensor::BusRecorder recorder(bus);
    recorder.start();
    name = recorder.filename();
    feed->push(makeScan(0));
  }
  std::filesystem::current_path(old_cwd);
  EXPECT_TRUE(name.starts_with("scan_"));
  EXPECT_TRUE(name.ends_with(".pbscan"));
  EXPECT_TRUE(std::filesystem::exists(dir / name));
  std::filesystem::remove_all(dir);
}

// A stalled disk: samples beyond the queues are dropped oldest first and
// counted; what was written plus what was dropped is what was published, and
// the newest samples are the ones kept. The drop is warned about as it
// happens, not only at the end.
TEST(BusRecorder, DropsAreCountedKeepTheNewestAndWarnedAboutAtOnce) {
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed);
  auto gate = std::make_shared<GatedFile>();
  TempFile file;
  LogCapture warnings(msensor::log::Level::warn);

  msensor::BusRecorder recorder(bus, {.scan_queue = 4, .imu_queue = 4}, gate);
  recorder.start(file.str());

  gate->closeGate();
  feed->push(makeScan(0)); // the writer takes it and blocks in write()
  ASSERT_TRUE(waitFor([&] { return gate->writerBlocked(); }));

  const auto begin = std::chrono::steady_clock::now();
  for (uint32_t i = 1; i <= 20; ++i) {
    feed->push(makeScan(i)); // 4 stay queued, 16 are dropped
  }
  for (uint32_t i = 0; i < 10; ++i) {
    feed->push(makeImu(i)); // 4 stay queued, 6 are dropped
  }
  // The warning arrives while the writer is still stuck.
  ASSERT_TRUE(waitFor([&] { return warnings.count() >= 2; }, 5s));
  const auto elapsed = std::chrono::steady_clock::now() - begin;
  // At most one per second per stream (two streams).
  const auto seconds =
      std::chrono::duration_cast<std::chrono::seconds>(elapsed);
  EXPECT_LE(warnings.count(), 2U * static_cast<size_t>(seconds.count() + 1));
  EXPECT_EQ(recorder.stats().lidar.dropped, 16U);
  EXPECT_EQ(recorder.stats().imu.dropped, 6U);

  gate->openGate();
  const auto stats = recorder.stop();
  EXPECT_EQ(stats.lidar.written, 5U);
  EXPECT_EQ(stats.lidar.dropped, 16U);
  EXPECT_EQ(stats.lidar.written + stats.lidar.dropped, 21U);
  EXPECT_EQ(stats.imu.written, 4U);
  EXPECT_EQ(stats.imu.dropped, 6U);

  const Contents got = readBack(file.str());
  EXPECT_EQ(got.scans, (std::vector<uint32_t>{0, 17, 18, 19, 20}));
  EXPECT_EQ(got.imus, (std::vector<uint32_t>{6, 7, 8, 9}));
}

// Stopping with samples still queued writes them all before closing.
TEST(BusRecorder, StopDrainsWhatIsStillQueued) {
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed);
  auto gate = std::make_shared<GatedFile>();
  TempFile file;
  msensor::BusRecorder recorder(bus, {}, gate);
  recorder.start(file.str());

  gate->closeGate();
  feed->push(makeScan(0));
  ASSERT_TRUE(waitFor([&] { return gate->writerBlocked(); }));
  for (uint32_t i = 1; i < 50; ++i) {
    feed->push(makeScan(i));
  }
  for (uint32_t i = 0; i < 200; ++i) {
    feed->push(makeImu(i));
  }

  // The disk comes back a moment after stop() has been asked to finish.
  std::thread reopen([&] {
    std::this_thread::sleep_for(100ms);
    gate->openGate();
  });
  const auto stats = recorder.stop();
  reopen.join();

  EXPECT_EQ(stats.lidar.written, 50U);
  EXPECT_EQ(stats.imu.written, 200U);
  EXPECT_EQ(stats.lidar.dropped + stats.imu.dropped, 0U);
  const Contents got = readBack(file.str());
  EXPECT_EQ(got.scans.size(), 50U);
  EXPECT_EQ(got.imus.size(), 200U);
  for (uint32_t i = 0; i < got.scans.size(); ++i) {
    ASSERT_EQ(got.scans[i], i);
  }
}

// Queue depths are the recorder's own, not the gRPC caps (64 scans, 4000 IMU).
TEST(BusRecorder, QueuesAreDeeperThanTheGrpcCaps) {
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed);
  auto gate = std::make_shared<GatedFile>();
  TempFile file;
  msensor::BusRecorder recorder(bus, {}, gate);
  recorder.start(file.str());
  gate->closeGate();
  feed->push(makeScan(0));
  ASSERT_TRUE(waitFor([&] { return gate->writerBlocked(); }));
  for (uint32_t i = 1; i <= 256; ++i) {
    feed->push(makeScan(i));
  }
  for (uint32_t i = 0; i < 8192; ++i) {
    feed->push(makeImu(i));
  }
  const auto live = recorder.stats();
  EXPECT_EQ(live.lidar.dropped, 0U); // 256 queued behind the entry in flight
  EXPECT_EQ(live.imu.dropped, 0U);
  gate->openGate();
  const auto stats = recorder.stop();
  EXPECT_EQ(stats.lidar.written, 257U);
  EXPECT_EQ(stats.imu.written, 8192U);
}

// A file that starts refusing writes is counted and logged, not fatal.
TEST(BusRecorder, WriteFailuresAreCountedAndTheRecorderStillStops) {
  class FailingFile : public msensor::IFile {
  public:
    void open(const std::string &) override {}
    void write(const char *, size_t) override {
      throw std::runtime_error("disk full");
    }
    void flush() override {}
    void close() override {}
  };
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed);
  LogCapture errors(msensor::log::Level::error);
  msensor::BusRecorder recorder(bus, {}, std::make_shared<FailingFile>());
  recorder.start("ignored");
  for (uint32_t i = 0; i < 5; ++i) {
    feed->push(makeScan(i));
  }
  const auto stats = recorder.stop();
  EXPECT_EQ(stats.lidar.written, 0U);
  EXPECT_EQ(stats.lidar.failed, 5U);
  EXPECT_GE(errors.count(), 1U);
}

// Stopping while the drivers are still publishing is safe, and the drivers
// can publish afterwards.
TEST(BusRecorder, StopWhileTheDriversKeepPublishing) {
  auto feed = std::make_shared<FeedSensor>();
  auto bus = busOf(feed);
  TempFile file;
  std::atomic<bool> run{true};
  std::thread producer([&] {
    uint32_t i = 0;
    while (run) {
      feed->push(makeImu(i));
      feed->push(makeScan(i));
      ++i;
      std::this_thread::sleep_for(50us);
    }
  });
  {
    msensor::BusRecorder recorder(bus);
    recorder.start(file.str());
    std::this_thread::sleep_for(100ms);
    const auto stats = recorder.stop();
    EXPECT_GT(stats.imu.written, 0U);
    EXPECT_GT(stats.lidar.written, 0U);
    EXPECT_FALSE(feed->hasImuConsumer());
  }
  std::this_thread::sleep_for(5ms);
  run = false;
  producer.join();
  const Contents got = readBack(file.str());
  for (size_t i = 1; i < got.imus.size(); ++i) {
    ASSERT_EQ(got.imus[i], got.imus[i - 1] + 1);
  }
}

// The config's defaults are the recorder's.
TEST(BusRecorder, ConfigDefaultsMatchTheRecorderDefaults) {
  const msensor::Config config;
  const msensor::BusRecorderOptions options;
  EXPECT_EQ(config.recording.scan_queue, options.scan_queue);
  EXPECT_EQ(config.recording.imu_queue, options.imu_queue);
}
