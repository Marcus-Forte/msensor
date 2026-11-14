#pragma once

#include <memory>

#include "msensor/interface/IAdc.hh"
#include "msensor/interface/ICamera.hh"
#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"

namespace msensor {

/// A set of sensors, e.g. what a SensorsServer publishes; a null member means
/// that sensor is not available.
struct Sensors {
  std::shared_ptr<IAdc> adc;
  std::shared_ptr<ICamera> camera;
  std::shared_ptr<IImu> imu;
  std::shared_ptr<ILidar> lidar;
};

} // namespace msensor
