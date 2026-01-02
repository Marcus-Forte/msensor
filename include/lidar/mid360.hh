#pragma once

#include <string>

#include "interface/IImu.hh"
#include "interface/ILidar.hh"

#include <boost/lockfree/spsc_queue.hpp>

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
   * data from getScan(). Typically the number of points per `getScan` is 96 *
   * `accumulate_scan_count`.
   */
  Mid360(const std::string &&config, size_t accumulate_scan_count);
  /// Initialize the Livox driver and connect to the device.
  void init() override;
  /// Retrieve the next accumulated point cloud.
  std::shared_ptr<Scan3DI> getScan() override;
  /// Retrieve the latest IMU sample from the embedded sensor.
  std::optional<IMUData> getImuData() override;
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
  std::shared_ptr<Scan3DI> accumulated_pointcloud_data_;

  boost::lockfree::spsc_queue<std::shared_ptr<Scan3DI>> scan_queue_;
  boost::lockfree::spsc_queue<IMUData> imu_queue_;

  const size_t accumulate_scan_count_;

  size_t scan_count_;

  uint32_t connection_handle_;
};

} // namespace msensor