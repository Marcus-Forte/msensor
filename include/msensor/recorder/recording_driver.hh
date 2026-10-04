#pragma once

#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"
#include "msensor/recorder/scan_player.hh"

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <thread>

namespace msensor {

/**
 * @brief Replays a recorded sensor file as push-based LiDAR and IMU sources.
 *
 * Entries are emitted in file order from a dedicated thread and paced against
 * the recorded timestamps scaled by a speed multiplier:
 *   - speed == 1.0: real time,
 *   - speed > 1.0 : faster than real time,
 *   - speed == 0.0: as fast as the recording can be read (no pacing).
 *
 * When the recording is exhausted isFinished() returns true. Unlike a hardware
 * driver, the driver does not know about the consumer's stream lifetime; the
 * owner (e.g. the playback server) is responsible for ending the stream.
 */
class RecordingSensorDriver : public ILidar, public IImu {
public:
  explicit RecordingSensorDriver(const std::filesystem::path &file,
                                 double speed = 1.0,
                                 bool start_paused = false);
  ~RecordingSensorDriver() override;

  void init() override {}

  void startSampling() override;
  void stopSampling() override;

  void setScanCallback(ScanCallback callback) override;
  void setImuCallback(ImuCallback callback) override;

  /// Pause or resume entry emission. Pacing excludes time spent paused.
  void setPaused(bool paused);
  bool isPaused() const;

  /// Double the playback multiplier; unpaced playback (speed 0) is unchanged.
  void increaseSpeed();
  /// Halve the multiplier down to 0.125x; from unpaced playback, set 64x.
  void decreaseSpeed();
  double speed() const;

  /// Rewind to the first entry on the sampling thread.
  void requestReset();

  /// True once the whole recording has been emitted.
  bool isFinished() const;

private:
  void run(std::stop_token st);
  void emitScan(const Scan3DI &scan);
  void emitImu(const IMUData &imu);

  ScanPlayer player_;
  std::atomic<double> speed_;
  std::jthread thread_;

  mutable std::mutex control_mutex_;
  std::condition_variable_any control_cv_;
  std::atomic<bool> paused_{false};
  std::atomic<bool> reset_requested_{false};

  std::mutex scan_mutex_;
  ScanCallback scan_callback_;
  std::mutex imu_mutex_;
  ImuCallback imu_callback_;

  std::atomic<bool> started_{false};
  std::atomic<bool> finished_{false};
};

} // namespace msensor
