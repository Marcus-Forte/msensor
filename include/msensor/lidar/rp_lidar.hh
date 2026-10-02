#pragma once

#include <optional>
#include <stop_token>
#include <string>
#include <thread>

#include "msensor/interface/CallbackSlot.hh"
#include "msensor/interface/ILidar.hh"
#include "sl_lidar_driver.h"

namespace msensor {

/**
 * @brief Wrapper around the RPLidar SDK providing the ILidar interface.
 */
class RPLidar : public ILidar {
public:
  RPLidar(const std::string &serial_port);
  virtual ~RPLidar();

  void init() override;
  /// Start publishing scans.
  void startSampling() override;
  /// Stop publishing scans.
  void stopSampling() override;
  /// Register the single scan consumer.
  void setScanCallback(ScanCallback callback) override;
  /// Configure motor speed in RPM.
  void setMotorRPM(unsigned int rpm);

private:
  /// Acquire a single scan from the sensor, or nullopt on timeout.
  std::optional<Scan3DI> grabScan();

  /// Sampling loop; runs on the sampling thread.
  void run(std::stop_token st);

  sl::IChannel *channel_;
  sl::ILidarDriver *drv_;
  uint32_t sequence_number_ = 0;
  CallbackSlot<Scan3DI> callback_;
  std::jthread thread_; // last: joined before the members above are destroyed
};

} // namespace msensor
