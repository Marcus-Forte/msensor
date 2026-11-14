#pragma once

#include "msensor/config/config.hh"
#include "msensor/interface/Sensors.hh"

namespace msensor {

/// Constructs, initialises and starts sampling the sensors selected by
/// `config`; a sensor that is not enabled (or, for the RPLidar, whose device
/// does not exist) is left null.
///
/// A sensor selected in `config.sim` replaces the hardware driver of the same
/// kind. The Mid360 provides the IMU too, so a standalone ICM20948 is skipped
/// when it is in use; if both Mid360 and RPLidar are enabled, the Mid360 wins.
///
/// Throws std::runtime_error if an enabled Mid360 has no config file, or a
/// driver cannot be created or initialised (e.g. a missing I2C bus).
Sensors makeSensors(const Config &config);

} // namespace msensor
