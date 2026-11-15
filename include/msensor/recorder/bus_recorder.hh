#pragma once

#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "msensor/bus/sensor_bus.hh"
#include "msensor/interface/IFile.hh"

namespace msensor {

/// Queue depths of a BusRecorder. They are not limited by the gRPC caps: a
/// recorder wants every sample, so the defaults hold about 12 s of Mid360
/// scans at 21 Hz and about 40 s of IMU samples at 200 Hz for a slow disk.
struct BusRecorderOptions {
  size_t scan_queue = 256; ///< LiDAR scans held while the writer is behind.
  size_t imu_queue = 8192; ///< IMU samples held while the writer is behind.
};

/// What a recorder did with one stream.
struct BusRecorderStreamStats {
  bool recorded = false; ///< The bus has this sensor, so it was subscribed.
  size_t written = 0;    ///< Entries appended to the file.
  size_t dropped = 0;    ///< Entries lost because the queue was full.
  size_t failed = 0;     ///< Entries the file refused (disk full, ...).
};

struct BusRecorderStats {
  BusRecorderStreamStats lidar;
  BusRecorderStreamStats imu;
};

/**
 * @brief Records the LiDAR and IMU topics of a SensorBus to a `.pbscan` file.
 *
 * start() subscribes to the bus's LiDAR and IMU topics, each with its own deep
 * queue, and starts one writer thread that appends the samples through
 * ScanRecorder, so the file format is unchanged and ScanPlayer reads it. The
 * drivers' threads only push into the queues; all serialization and disk work
 * runs on the writer thread. Either sensor may be absent from the bus; at
 * least one must be present.
 *
 * At each wake-up the writer drains the IMU samples queued at that moment
 * before the queued scans, so the IMU samples of a scan precede it in the file
 * as they do in arrival order. When a queue is full the oldest sample is lost
 * and counted; that is logged as a warning at once, at most once per second
 * per stream.
 *
 * stop() writes what is queued, closes the file and logs the entries written
 * and dropped per stream. Stop it before the bus (and so the drivers) are
 * destroyed; the destructor calls it too. Samples that arrive while stop() is
 * draining may be left out. start(), stop() and stats() are meant to be called
 * from one thread (they are serialized by a mutex).
 */
class BusRecorder {
public:
  /// `file` defaults to a msensor::File; a test may pass its own.
  explicit BusRecorder(std::shared_ptr<SensorBus> bus,
                       BusRecorderOptions options = {},
                       std::shared_ptr<IFile> file = nullptr);
  ~BusRecorder();

  BusRecorder(const BusRecorder &) = delete;
  BusRecorder &operator=(const BusRecorder &) = delete;

  /// Opens (truncates) `filename` and starts recording; an empty name is
  /// "scan_<timestamp>.pbscan" in the working directory. Throws
  /// std::runtime_error if the file cannot be opened, and
  /// std::logic_error if already started or if the bus has neither a LiDAR nor
  /// an IMU topic. A recorder records once: it cannot be restarted after
  /// stop().
  void start(const std::string &filename = {});

  /// Drains the queues, closes the file and returns the final counts. Does
  /// nothing (and returns the last counts) when not running.
  BusRecorderStats stop();

  /// Counts so far; final after stop().
  BusRecorderStats stats() const;

  bool running() const;

  /// The file being written (empty before start()).
  std::string filename() const;

private:
  void run(std::stop_token st);
  void drain();
  void wake();

  const std::shared_ptr<SensorBus> bus_;
  BusRecorderOptions options_;

  mutable std::mutex control_m_; // start/stop/stats
  bool started_ = false;
  bool stopped_ = false;
  std::string filename_;
  BusRecorderStats final_stats_;

  std::mutex wake_m_;
  std::condition_variable_any wake_cv_;
  bool wake_ = false; // guarded by wake_m_

  struct Impl;
  std::unique_ptr<Impl>
      impl_;            // streams, ScanRecorder; destroyed after the thread
  std::jthread writer_; // last: joined first
};

} // namespace msensor
