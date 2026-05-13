#pragma once

#include <optional>
#include <stdint.h>

#include "interface/Header.hh"

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
  /**
   * @brief Retrieve the latest IMU sample if available.
   *
   * @return std::optional<IMUData> Most recent IMU reading, or empty if none
   *         is ready.
   */
  virtual std::optional<IMUData> getImuData() = 0;
};

} // namespace msensor
