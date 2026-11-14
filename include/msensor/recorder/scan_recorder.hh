#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "msensor/interface/IFile.hh"
#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"

namespace sensors {
class RecordingEntry;
}

namespace msensor {

/**
 * @brief Thread-safe recorder for LiDAR scans and IMU samples.
 *
 * Each record() appends one entry, a native-endian `uint64_t` byte count
 * followed by a serialized `sensors::RecordingEntry`, and flushes the file, so
 * a killed recorder loses at most the entry being written. ScanPlayer reads
 * this format. All members are guarded by one mutex per recorder, so a LiDAR
 * and an IMU thread can record concurrently.
 */
class ScanRecorder {
public:
  /// Create a recorder that writes into the provided file adapter.
  ScanRecorder(const std::shared_ptr<IFile> &file);
  ~ScanRecorder();

  ScanRecorder(const ScanRecorder &) = delete;
  ScanRecorder &operator=(const ScanRecorder &) = delete;

  /**
   * @brief Start the recording. Creates a file with the current timestamp.
   *
   */
  void start();

  /**
   * @brief Start the recording. Creates a file with a given name.
   *
   * @param filename
   */
  void start(const std::string &filename);

  /**
   * @brief Records a laser scan into scanfile. Thread-safe. Ignored unless
   * the recording has started.
   *
   */
  void record(const Scan3DI &scan);

  /**
   * @brief Records an IMU data into scanfile. Thread-safe. Ignored unless the
   * recording has started.
   *
   */
  void record(const IMUData &imu);

  /**
   * @brief Stops the recording.
   *
   */
  void stop();

  /// Return the current output filename.
  const std::string &getFilename() const;

private:
  /// Serialize `entry` once into the reused buffer and append it to the file.
  /// Caller holds mutex_.
  void append(const sensors::RecordingEntry &entry);

  std::shared_ptr<IFile> record_file_;
  std::mutex mutex_;
  bool has_started_ = false;
  std::string filename_;
  // One entry per kind, so the oneof never switches and the scan's repeated
  // fields keep their capacity between records.
  std::unique_ptr<sensors::RecordingEntry> scan_entry_;
  std::unique_ptr<sensors::RecordingEntry> imu_entry_;
  std::vector<std::uint8_t> buffer_;
};

} // namespace msensor
