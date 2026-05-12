#include <filesystem>
#include <iostream>
#include <string_view>
#include <thread>

#include "adc/ADS1115.hh"
#include "camera/opencv_camera.hh"
#include "config/config.hh"
#include "imu/icm-20948.h"
#include "imu/icm-20948_defs.h"
#include "lidar/mid360.hh"
#include "lidar/rp_lidar.hh"
#include "msensor_server.hh"

constexpr int DefaultI2cBus = 1;
constexpr uint8_t DefaultADSAddress = 0x48;

static void print_usage() {
  std::cout
      << "Usage: sensor_publisher [config.json]"
      << std::endl;
}

int main(int argc, char **argv) {
  if (argc > 2) {
    print_usage();
    return 1;
  }

  if (argc == 2 && std::string_view(argv[1]) == "-h") {
    print_usage();
    return 0;
  }

  const std::filesystem::path config_path =
      argc == 2 ? std::filesystem::path(argv[1]) : msensor::Config::defaultConfigPath();
    const msensor::Config config = msensor::Config::fromFile(config_path);

  std::shared_ptr<msensor::ILidar> lidar = nullptr;
  std::shared_ptr<msensor::IImu> imu = nullptr;

  if (config.mid360.enable && config.rplidar.enable) {
    std::cerr << "Both mid360 and rplidar are enabled; preferring mid360."
              << std::endl;
  }

  if (config.mid360.enable) {
    if (config.mid360.config.empty()) {
      std::cerr << "mid360 is enabled but no config path was provided. "
                   "Exiting."
                << std::endl;
      return 1;
    } else if (!std::filesystem::exists(config.mid360.config)) {
      std::cerr << "mid360 config file: " << config.mid360.config
                << " does not exist. Exiting."
                << std::endl;
      return 1;
    } else {
      auto mid360 = std::make_shared<msensor::Mid360>(
          std::string(config.mid360.config), 100);
      mid360->init();
      mid360->setMode(msensor::Mid360::Mode::Normal);
      mid360->setScanPattern(msensor::Mid360::ScanPattern::NonRepetitive);
      mid360->startSampling();
      lidar = mid360;
      if (config.icm20948.enable) {
        std::cerr << "mid360 already provides IMU data; skipping standalone "
                     "ICM20948 initialization."
                  << std::endl;
      }
      imu = mid360;
    }
  } else if (config.rplidar.enable) {
    if (std::filesystem::exists(config.rplidar.device)) {
      auto rp = std::make_shared<msensor::RPLidar>(config.rplidar.device);
      rp->init();
      rp->setMotorRPM(360);
      lidar = rp;
    } else {
      std::cerr << "Lidar device: " << config.rplidar.device
                << " does not exist. Lidar will be unavailable." << std::endl;
    }
  }

  constexpr float external_gain =
      (10.0f + 5.1f) / 5.1f; // 10k and 5.1k resistors

  std::shared_ptr<msensor::IAdc> adc = nullptr;
  if (config.ads1115.enable) {
    auto ads1115 =
        std::make_shared<msensor::ADS1115>(config.ads1115.i2c_bus,
                                           DefaultADSAddress);
    ads1115->init(msensor::ADS1115::Gain::PLUS_MINUS_6_144,
                  msensor::ADS1115::DataRate::SPS_8,
                  static_cast<msensor::ADS1115::Channel>(0), external_gain);
    adc = ads1115;
  }

  if (config.icm20948.enable && !imu) {
    auto icm20948 =
        std::make_shared<msensor::ICM20948>(config.ads1115.i2c_bus,
                                            ICM20948_ADDR0);
    icm20948->init();
    icm20948->calibrate();
    imu = icm20948;
  }

  std::shared_ptr<msensor::ICamera> camera = nullptr;
  if (config.camera.enable) {
    std::cout << "Using camera pipeline: " << config.camera.pipeline
              << std::endl;
    camera = std::make_shared<msensor::OpenCvCamera>(
        std::string(config.camera.pipeline));
  }

  SensorsServer server(adc, camera, imu, lidar);
  server.start();

  while (true) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}