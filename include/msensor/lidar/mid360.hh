#pragma once

#include <string>

#include "msensor/interface/CallbackSlot.hh"
#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"

#include <atomic>

namespace msensor {

/**
 * @brief This class represents a Mid360 lidar, with getters methods to receive
 * LiDAR data. It has an embedded IMU sensor, therefore IImu is inherited as
 * well.
 *
 * \note Manual:
 * https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html#point-cloud-imu-data-protocol
 *
 */
class Mid360 : public ILidar, public IImu {
public:
  enum class ScanPattern { Repetitive, NonRepetitive, LowFrameRate };
  enum class Mode { Normal, PowerSave };

  /**
   * @brief Construct a new Mid360 object.
   *
   * @param config Configuration file to be loaded. \note The IP address of the
   * LiDAR is one of the configuration elements. Make sure your machine lies
   * within a reacheable subnet of the LiDAR.
   * @param accumulate_scan_count number of samples to accumulate when returning
   * data per published scan. Typically the number of points per scan is 96 *
   * `accumulate_scan_count`.
   */
  Mid360(std::string config, size_t accumulate_scan_count);
  ~Mid360() override;
  /// Initialize the Livox driver and connect to the device.
  void init() override;

  /// Register the single consumer of accumulated point clouds. The callback
  /// runs on the Livox SDK point-cloud callback thread.
  /// \note Time is in nanoseconds and corresponds to the first point of the
  /// first UDP packet accumulated into the scan.
  void setScanCallback(ScanCallback callback) override;

  /// Register the single consumer of IMU samples from the embedded sensor.
  /// \note Time is in nanoseconds.
  void setImuCallback(ImuCallback callback) override;

  /// Start sampling LiDAR and IMU data.
  void startSampling() override;

  /// Stop sampling operations.
  void stopSampling() override;

  /// Switch power/normal operating mode.
  void setMode(Mode mode);

  /// Configure point emission pattern.
  void setScanPattern(ScanPattern pattern) const;

private:
  const std::string config_;
  Scan3DI accumulated_scan_;
  size_t packets_in_scan_{0};

  CallbackSlot<Scan3DI> scan_callback_;
  CallbackSlot<IMUData> imu_callback_;

  std::atomic<uint32_t> scan_sequence_{0};
  std::atomic<uint32_t> imu_sequence_{0};

  const size_t accumulate_scan_count_;

  uint32_t connection_handle_{0};
  bool sdk_initialized_{false};
  bool sampling_{false};
};

} // namespace msensor