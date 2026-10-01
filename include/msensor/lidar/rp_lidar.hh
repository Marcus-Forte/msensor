#pragma once

#include "msensor/hub/PollingProducer.hh"
#include "msensor/interface/ILidar.hh"
#include "sl_lidar_driver.h"
#include <string>

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
  void startSampling() override { producer_.start(); }
  /// Stop publishing scans.
  void stopSampling() override { producer_.stop(); }
  SensorHub<Scan3DI> &scans() override { return hub_; }
  /// Configure motor speed in RPM.
  void setMotorRPM(unsigned int rpm);

private:
  /// Acquire a single scan from the sensor, or nullptr on timeout.
  std::shared_ptr<const Scan3DI> grabScan();

  sl::IChannel *channel_;
  sl::ILidarDriver *drv_;
  uint32_t sequence_number_ = 0;
  SensorHub<Scan3DI> hub_;
  PollingProducer<Scan3DI> producer_;
};

} // namespace msensor