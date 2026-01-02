#pragma once

#include "interface/ILidar.hh"
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
  inline void startSampling() override {}
  inline void stopSampling() override {}
  /// Acquire a single scan from the sensor.
  std::shared_ptr<Scan3DI> getScan() override;
  /// Configure motor speed in RPM.
  void setMotorRPM(unsigned int rpm);

private:
  sl::IChannel *channel_;
  sl::ILidarDriver *drv_;
};

} // namespace msensor