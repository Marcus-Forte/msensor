#pragma once

#include <functional>
#include <stdint.h>

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
  /// Callback invoked for every produced sample.
  using ImuCallback = std::function<void(const IMUData &)>;

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
   * @brief Register the single consumer of IMU samples.
   *
   * The callback runs on the driver's sampling thread. Passing an empty
   * callback clears it. There is at most one consumer per stream.
   */
  virtual void setImuCallback(ImuCallback callback) = 0;
};

} // namespace msensor
