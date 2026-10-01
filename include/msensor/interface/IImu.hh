#pragma once

#include <stdint.h>

#include "msensor/hub/SensorHub.hh"
#include "msensor/interface/Header.hh"

namespace msensor {

/**
 * @brief IMU sample containing accelerometer and gyroscope data.
 */
struct IMUData {
  Header header;
  float ax; ///< Linear acceleration on X (m/s^2).
  float ay; ///< Linear acceleration on Y (m/s^2).
  float az; ///< Linear acceleration on Z (m/s^2).
  float gx; ///< Angular velocity around X (rad/s).
  float gy; ///< Angular velocity around Y (rad/s).
  float gz; ///< Angular velocity around Z (rad/s).
};

/**
 * @brief Interface for IMU data providers.
 */
class IImu {
public:
  virtual ~IImu() = default;

  /**
   * @brief Start publishing IMU samples.
   */
  virtual void startSampling() = 0;
  /**
   * @brief Stop publishing IMU samples.
   */
  virtual void stopSampling() = 0;

  /**
   * @brief Hub publishing IMU samples.
   *
   * Subscribe to receive every sample independently of other consumers.
   */
  virtual SensorHub<IMUData> &imu() = 0;
};

} // namespace msensor
