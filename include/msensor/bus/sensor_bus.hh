#pragma once

#include <memory>
#include <utility>

#include "msensor/bus/topic.hh"
#include "msensor/interface/Sensors.hh"

namespace msensor {

/**
 * @brief The topics of a set of sensors.
 *
 * Built from a `Sensors` struct: one topic per available streaming sensor
 * (LiDAR scans, IMU samples, camera frames; the ADC is pull-based and has
 * none). It keeps the drivers alive and, lazily, becomes each driver's single
 * callback consumer while the topic has subscribers. Share one bus between the
 * gRPC server and any other consumer with a `std::shared_ptr<SensorBus>`.
 *
 * A bus must not be built over drivers that already have another consumer:
 * the topic replaces it while it has subscribers and clears it afterwards.
 */
class SensorBus {
public:
  explicit SensorBus(Sensors sensors) : sensors_(std::move(sensors)) {
    if (sensors_.lidar) {
      lidar_ = std::make_unique<Topic<Scan3DI>>(
          "lidar", [driver = sensors_.lidar](Topic<Scan3DI>::Callback cb) {
            driver->setScanCallback(std::move(cb));
          });
    }
    if (sensors_.imu) {
      imu_ = std::make_unique<Topic<IMUData>>(
          "imu", [driver = sensors_.imu](Topic<IMUData>::Callback cb) {
            driver->setImuCallback(std::move(cb));
          });
    }
    if (sensors_.camera) {
      camera_ = std::make_unique<Topic<CameraFrame>>(
          "camera",
          [driver = sensors_.camera](Topic<CameraFrame>::Callback cb) {
            driver->setFrameCallback(std::move(cb));
          });
    }
  }

  SensorBus(const SensorBus &) = delete;
  SensorBus &operator=(const SensorBus &) = delete;

  /// The topic of each sensor, or null when the sensor is not available.
  /// The pointers stay valid as long as the bus lives.
  Topic<Scan3DI> *lidar() const noexcept { return lidar_.get(); }
  Topic<IMUData> *imu() const noexcept { return imu_.get(); }
  Topic<CameraFrame> *camera() const noexcept { return camera_.get(); }

  /// The sensors the bus was built from (the ADC is not on the bus).
  const Sensors &sensors() const noexcept { return sensors_; }

private:
  Sensors sensors_;
  std::unique_ptr<Topic<Scan3DI>> lidar_;
  std::unique_ptr<Topic<IMUData>> imu_;
  std::unique_ptr<Topic<CameraFrame>> camera_;
};

} // namespace msensor
