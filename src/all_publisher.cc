#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <thread>

#include "adc/ADS1115.hh"
#include "camera/opencv_camera.hh"
#include "imu/icm-20948.h"
#include "imu/icm-20948_defs.h"
#include "lidar/rp_lidar.hh"
#include "sensors_server.hh"

constexpr int DefaultI2cBus = 1;
constexpr uint8_t DefaultADSAddress = 0x48;
constexpr const char *DefaultLidarDevice = "/dev/ttyUSB0";
constexpr uint64_t LoopPeriodUs = 1000;
constexpr const char *DefaultStreamPipeline =
    "libcamerasrc ! "
    "video/x-raw,format=BGR,width=1536,height=864,framerate=10/1 ! "
    "videoconvert ! appsink max-buffers=1 drop=true sync=false";

static void print_usage() {
  std::cout
      << "Usage: all_publisher [-b <i2c bus>] [-l <usb lidar device path>]"
      << std::endl;
}

int main(int argc, char **argv) {

  int opt;
  int bus = DefaultI2cBus;
  std::filesystem::path lidar_device(DefaultLidarDevice);

  while ((opt = getopt(argc, argv, "b:l:h")) != -1) {
    switch (opt) {
    case 'b':
      bus = std::strtol(optarg, nullptr, 0);
      break;
    case 'l':
      lidar_device = std::filesystem::path(optarg);
      break;
    case 'h':
    default:
      print_usage();
      return 0;
    }
  }

  std::shared_ptr<RPLidar> lidar = nullptr;
  if (std::filesystem::exists(lidar_device)) {
    lidar = std::make_shared<msensor::RPLidar>(lidar_device);
    lidar->init();
    lidar->setMotorRPM(360);
  } else {
    std::cerr << "Lidar device: " << lidar_device
              << " does not exist. Lidar will be unavailable." << std::endl;
  }

  constexpr float external_gain =
      (10.0f + 5.1f) / 5.1f; // 10k and 5.1k resistors

  const auto ads1115 =
      std::make_shared<msensor::ADS1115>(bus, DefaultADSAddress);
  ads1115->init(msensor::ADS1115::Gain::PLUS_MINUS_6_144,
                msensor::ADS1115::DataRate::SPS_8,
                static_cast<msensor::ADS1115::Channel>(0), external_gain);

  const auto icm20948 =
      std::make_shared<msensor::ICM20948>(bus, ICM20948_ADDR0);
  icm20948->init();
  icm20948->calibrate();

  const char *env_pipeline = std::getenv("CAMERA_PIPELINE");
  const char *pipeline =
      env_pipeline != nullptr ? env_pipeline : DefaultStreamPipeline;
  std::cout << "Using camera pipeline: " << pipeline << std::endl;
  const auto camera = std::make_shared<msensor::OpenCvCamera>(pipeline);

  SensorsServer server(ads1115, camera, icm20948, lidar);
  server.start();

  while (true) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}