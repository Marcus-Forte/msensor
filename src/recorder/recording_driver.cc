#include "msensor/recorder/recording_driver.hh"

#include "msensor/conversions/conversions.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace msensor {

RecordingSensorDriver::RecordingSensorDriver(const std::filesystem::path &file,
                                             double speed, bool start_paused)
    : player_(file), speed_(speed), paused_(start_paused) {
  if (!std::isfinite(speed) || speed < 0.0) {
    throw std::invalid_argument("Playback speed must be non-negative");
  }
}

RecordingSensorDriver::~RecordingSensorDriver() { stopSampling(); }

void RecordingSensorDriver::startSampling() {
  if (started_.exchange(true)) {
    return;
  }
  thread_ = std::jthread([this](std::stop_token st) { run(st); });
}

void RecordingSensorDriver::stopSampling() {
  if (thread_.joinable()) {
    thread_.request_stop();
    thread_.join();
  }
}

void RecordingSensorDriver::setScanCallback(ScanCallback callback) {
  std::lock_guard lock(scan_mutex_);
  scan_callback_ = std::move(callback);
}

void RecordingSensorDriver::setImuCallback(ImuCallback callback) {
  std::lock_guard lock(imu_mutex_);
  imu_callback_ = std::move(callback);
}

void RecordingSensorDriver::setPaused(bool paused) {
  {
    std::lock_guard lock(control_mutex_);
    paused_.store(paused);
  }
  control_cv_.notify_all();
}

bool RecordingSensorDriver::isPaused() const { return paused_.load(); }

void RecordingSensorDriver::increaseSpeed() {
  constexpr double kMaxSpeed = 1024.0;
  bool changed = false;
  {
    std::lock_guard lock(control_mutex_);
    const double current = speed_.load();
    if (current > 0.0 && current < kMaxSpeed) {
      speed_.store(std::min(current * 2.0, kMaxSpeed));
      changed = true;
    }
  }
  if (changed) {
    control_cv_.notify_all();
  }
}

void RecordingSensorDriver::decreaseSpeed() {
  constexpr double kMinSpeed = 0.125;
  constexpr double kSpeedFromUnpaced = 64.0;
  bool changed = false;
  {
    std::lock_guard lock(control_mutex_);
    const double current = speed_.load();
    if (current == 0.0) {
      speed_.store(kSpeedFromUnpaced);
      changed = true;
    } else if (current > kMinSpeed) {
      speed_.store(std::max(current / 2.0, kMinSpeed));
      changed = true;
    }
  }
  if (changed) {
    control_cv_.notify_all();
  }
}

double RecordingSensorDriver::speed() const { return speed_.load(); }

void RecordingSensorDriver::requestReset() {
  {
    std::lock_guard lock(control_mutex_);
    reset_requested_.store(true);
  }
  control_cv_.notify_all();
}

bool RecordingSensorDriver::isFinished() const { return finished_.load(); }

void RecordingSensorDriver::run(std::stop_token st) {
  auto wall_start = std::chrono::steady_clock::now();
  double pacing_speed = speed_.load();
  bool have_origin = false;
  uint64_t origin_ns = 0;
  bool have_emitted_timestamp = false;
  uint64_t last_emitted_timestamp_ns = 0;

  // Preserve the media-time progress made at the old speed when changing the
  // wall-clock mapping. Unpaced playback anchors from the last emitted entry.
  const auto rebase_pacing = [&](double new_speed) {
    const auto now = std::chrono::steady_clock::now();
    if (have_origin) {
      if (pacing_speed > 0.0) {
        const long double elapsed_ns =
            std::chrono::duration<long double, std::nano>(now - wall_start)
                .count();
        const long double advanced_ns =
            static_cast<long double>(origin_ns) + elapsed_ns * pacing_speed;
        origin_ns = static_cast<uint64_t>(std::min(
            advanced_ns,
            static_cast<long double>(std::numeric_limits<uint64_t>::max())));
      } else if (have_emitted_timestamp) {
        origin_ns = last_emitted_timestamp_ns;
      }
      wall_start = now;
    }
    pacing_speed = new_speed;
  };

  const auto reset_playback = [&]() {
    player_.reset();
    wall_start = std::chrono::steady_clock::now();
    pacing_speed = speed_.load();
    have_origin = false;
    origin_ns = 0;
    have_emitted_timestamp = false;
    last_emitted_timestamp_ns = 0;
    finished_.store(false);
  };

  while (!st.stop_requested()) {
    bool reset_requested;
    {
      std::lock_guard lock(control_mutex_);
      reset_requested = reset_requested_.exchange(false);
    }
    if (reset_requested) {
      reset_playback();
    }

    const double observed_speed = speed_.load();
    if (observed_speed != pacing_speed) {
      rebase_pacing(observed_speed);
    }

    if (paused_.load()) {
      const auto pause_start = std::chrono::steady_clock::now();
      std::unique_lock lock(control_mutex_);
      control_cv_.wait(lock, st, [this]() {
        return !paused_.load() || reset_requested_.load();
      });
      if (!reset_requested_.load()) {
        wall_start += std::chrono::steady_clock::now() - pause_start;
      }
      continue;
    }

    if (!player_.next()) {
      std::unique_lock lock(control_mutex_);
      paused_.store(true);
      finished_.store(true);
      control_cv_.wait(lock, st, [this]() { return reset_requested_.load(); });
      continue;
    }

    const auto &entry = player_.getLastEntry();

    uint64_t timestamp_ns = 0;
    switch (entry.entry_case()) {
    case sensors::RecordingEntry::kScan:
      timestamp_ns = entry.scan().header().timestamp();
      break;
    case sensors::RecordingEntry::kImu:
      timestamp_ns = entry.imu().header().timestamp();
      break;
    default:
      continue;
    }

    if (!have_origin) {
      origin_ns = timestamp_ns;
      have_origin = true;
    }

    bool restart_entry = false;
    while (!st.stop_requested()) {
      const double observed_speed = speed_.load();
      if (observed_speed != pacing_speed) {
        rebase_pacing(observed_speed);
      }

      bool reset_requested;
      {
        std::lock_guard lock(control_mutex_);
        reset_requested = reset_requested_.exchange(false);
      }
      if (reset_requested) {
        reset_playback();
        restart_entry = true;
        break;
      }

      if (paused_.load()) {
        const auto pause_start = std::chrono::steady_clock::now();
        std::unique_lock lock(control_mutex_);
        control_cv_.wait(lock, st, [this]() {
          return !paused_.load() || reset_requested_.load();
        });
        if (reset_requested_.exchange(false)) {
          reset_playback();
          restart_entry = true;
          break;
        }
        wall_start += std::chrono::steady_clock::now() - pause_start;
        continue;
      }

      const double current_speed = pacing_speed;
      if (current_speed <= 0.0 || timestamp_ns <= origin_ns) {
        break;
      }

      const auto offset = std::chrono::nanoseconds(static_cast<int64_t>(
          static_cast<double>(timestamp_ns - origin_ns) / current_speed));
      const auto deadline = wall_start + offset;
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }

      std::unique_lock lock(control_mutex_);
      control_cv_.wait_until(lock, st, deadline, [this, current_speed]() {
        return paused_.load() || reset_requested_.load() ||
               speed_.load() != current_speed;
      });
    }

    if (restart_entry) {
      continue;
    }
    {
      std::lock_guard lock(control_mutex_);
      reset_requested = reset_requested_.exchange(false);
    }
    if (reset_requested) {
      reset_playback();
      continue;
    }
    if (st.stop_requested()) {
      break;
    }

    switch (entry.entry_case()) {
    case sensors::RecordingEntry::kScan:
      emitScan(fromProtobuf(entry.scan()));
      break;
    case sensors::RecordingEntry::kImu:
      emitImu(fromProtobuf(entry.imu()));
      break;
    default:
      break;
    }
    last_emitted_timestamp_ns = timestamp_ns;
    have_emitted_timestamp = true;
  }

  finished_.store(true);
}

void RecordingSensorDriver::emitScan(const Scan3DI &scan) {
  std::lock_guard lock(scan_mutex_);
  if (scan_callback_) {
    scan_callback_(scan);
  }
}

void RecordingSensorDriver::emitImu(const IMUData &imu) {
  std::lock_guard lock(imu_mutex_);
  if (imu_callback_) {
    imu_callback_(imu);
  }
}

} // namespace msensor
