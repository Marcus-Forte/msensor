#include "msensor/factory/sensor_factory.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>

using namespace std::chrono_literals;

namespace {

/// A config with every hardware sensor disabled and no simulator selected.
msensor::Config noSensors() {
  msensor::Config config;
  config.rplidar.enable = false;
  config.camera.enable = false;
  config.mid360.enable = false;
  config.icm20948.enable = false;
  config.ads1115.enable = false;
  return config;
}

/// Waits for the first delivery of a callback-based sensor. The state is shared
/// so a late callback never touches a destroyed test frame.
struct Gate {
  std::mutex m;
  std::condition_variable cv;
  bool fired = false;
  void fire() {
    std::lock_guard lock(m);
    fired = true;
    cv.notify_all();
  }
  bool wait() {
    std::unique_lock lock(m);
    return cv.wait_for(lock, 10s, [&] { return fired; });
  }
};

/// An I2C bus number with no device node, or -1 if there is none.
int missingI2cBus() {
  const int bus = 987;
  return std::filesystem::exists("/dev/i2c-" + std::to_string(bus)) ? -1 : bus;
}

TEST(SensorFactory, SimulatorsDeliverData) {
  msensor::Config config = noSensors();
  config.sim = {.lidar = true, .imu = true, .camera = true, .adc = true};

  msensor::Sensors sensors = msensor::makeSensors(config);
  ASSERT_TRUE(sensors.lidar);
  ASSERT_TRUE(sensors.imu);
  ASSERT_TRUE(sensors.camera);
  ASSERT_TRUE(sensors.adc);

  auto scan = std::make_shared<Gate>();
  auto imu = std::make_shared<Gate>();
  auto frame = std::make_shared<Gate>();
  sensors.lidar->setScanCallback([scan](msensor::Scan3DI) { scan->fire(); });
  sensors.imu->setImuCallback([imu](msensor::IMUData) { imu->fire(); });
  sensors.camera->setFrameCallback(
      [frame](msensor::CameraFrame) { frame->fire(); });

  EXPECT_TRUE(scan->wait());
  EXPECT_TRUE(imu->wait());
  EXPECT_TRUE(frame->wait());
  EXPECT_TRUE(sensors.adc->readSingleEnded().has_value());

  sensors.lidar->stopSampling();
  sensors.imu->stopSampling();
  sensors.camera->stopSampling();
}

TEST(SensorFactory, NothingEnabledGivesNullSensors) {
  const msensor::Sensors sensors = msensor::makeSensors(noSensors());
  EXPECT_FALSE(sensors.lidar);
  EXPECT_FALSE(sensors.imu);
  EXPECT_FALSE(sensors.camera);
  EXPECT_FALSE(sensors.adc);
}

TEST(SensorFactory, SimLidarOverridesHardwareLidar) {
  msensor::Config config = noSensors();
  config.rplidar.enable = true;
  config.rplidar.device = "/dev/does_not_exist_msensor";
  config.sim.lidar = true;
  msensor::Sensors sensors;
  ASSERT_NO_THROW(sensors = msensor::makeSensors(config));
  EXPECT_TRUE(sensors.lidar);
  sensors.lidar->stopSampling();
}

TEST(SensorFactory, MissingRplidarDeviceLeavesLidarNull) {
  msensor::Config config = noSensors();
  config.rplidar.enable = true;
  config.rplidar.device = "/dev/does_not_exist_msensor";
  msensor::Sensors sensors;
  ASSERT_NO_THROW(sensors = msensor::makeSensors(config));
  EXPECT_FALSE(sensors.lidar);
}

TEST(SensorFactory, Mid360WithoutConfigPathThrows) {
  msensor::Config config = noSensors();
  config.mid360.enable = true;
  config.mid360.config = "";
  EXPECT_THROW(msensor::makeSensors(config), std::runtime_error);
}

TEST(SensorFactory, Mid360WithMissingConfigFileThrows) {
  msensor::Config config = noSensors();
  config.mid360.enable = true;
  config.mid360.config = "/nonexistent/msensor/mid360.json";
  EXPECT_THROW(msensor::makeSensors(config), std::runtime_error);
}

TEST(SensorFactory, Ads1115OnMissingBusThrows) {
  const int bus = missingI2cBus();
  if (bus < 0) GTEST_SKIP() << "I2C bus unexpectedly exists";
  msensor::Config config = noSensors();
  config.ads1115.enable = true;
  config.ads1115.i2c_bus = bus;
  EXPECT_THROW(msensor::makeSensors(config), std::runtime_error);
}

TEST(SensorFactory, Icm20948OnMissingBusThrows) {
  const int bus = missingI2cBus();
  if (bus < 0) GTEST_SKIP() << "I2C bus unexpectedly exists";
  msensor::Config config = noSensors();
  config.icm20948.enable = true;
  config.icm20948.i2c_bus = bus;
  EXPECT_THROW(msensor::makeSensors(config), std::runtime_error);
}

TEST(SensorFactory, SimImuOverridesIcm20948) {
  const int bus = missingI2cBus();
  if (bus < 0) GTEST_SKIP() << "I2C bus unexpectedly exists";
  msensor::Config config = noSensors();
  config.icm20948.enable = true;
  config.icm20948.i2c_bus = bus;
  config.sim.imu = true;
  msensor::Sensors sensors;
  ASSERT_NO_THROW(sensors = msensor::makeSensors(config));
  EXPECT_TRUE(sensors.imu);
  sensors.imu->stopSampling();
}

} // namespace
