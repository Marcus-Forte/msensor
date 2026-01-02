#include <filesystem>
#include <iostream>
#include <thread>

#include "adc/ADS1115.hh"
#include "camera/opencv_camera.hh"
#include "imu/icm-20948.h"
#include "imu/icm-20948_defs.h"
#include "lidar/rp_lidar.hh"
#include "sensors_server.hh"
#include "timing/timing.hh"

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

  if (!std::filesystem::exists(lidar_device)) {
    std::cerr << "Lidar device path does not exist: " << lidar_device
              << std::endl;
    return -1;
  }

  auto ads1115 = std::make_shared<msensor::ADS1115>(bus, DefaultADSAddress);
  ads1115->init(msensor::ADS1115::Gain::PLUS_MINUS_6_144,
               msensor::ADS1115::DataRate::SPS_8,
               static_cast<msensor::ADS1115::Channel>(0));

  auto rplidar = std::make_shared<msensor::RPLidar>(lidar_device);
  rplidar->init();
  rplidar->setMotorRPM(360);

  auto icm20948 = std::make_shared<msensor::ICM20948>(bus, ICM20948_ADDR0);
  icm20948->init();
  icm20948->calibrate();

  auto camera = std::make_shared<msensor::OpenCvCamera>(DefaultStreamPipeline);

  SensorsServer server(ads1115, camera, icm20948, rplidar);
  server.start();

  while (true) {

    // constexpr float ExternalGain =
    //     (10.0f + 5.1f) / 5.1f; // 10k and 5.1k resistors
    // adc_data->voltage *= ExternalGain;

    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}