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

  msensor::ADS1115 ads1115(bus, DefaultADSAddress);
  ads1115.init(msensor::ADS1115::Gain::PLUS_MINUS_6_144,
               msensor::ADS1115::DataRate::SPS_8,
               static_cast<msensor::ADS1115::Channel>(0));

  msensor::RPLidar rplidar(lidar_device);
  rplidar.init();
  rplidar.setMotorRPM(360);

  msensor::ICM20948 icm20948(bus, ICM20948_ADDR0);
  icm20948.init();
  icm20948.calibrate();

  msensor::OpenCvCamera camera(DefaultStreamPipeline);

  SensorsServer server;
  server.start();

  cv::Mat frame;

  while (true) {
    const auto now = timing::getNowUs();

    const auto scan = rplidar.getScan();
    const auto imudata = icm20948.getImuData();
    auto adc_data = ads1115.readSingleEnded();

    constexpr float ExternalGain =
        (10.0f + 5.1f) / 5.1f; // 10k and 5.1k resistors
    adc_data->voltage *= ExternalGain;

    if (scan) {
      server.publishScan(scan);
      // std::cout << "New Scan @ " << scan->timestamp
      //           << " Points: " << scan->points->size() << std::endl;
    }
    if (imudata) {
      server.publishImu(imudata.value());
    }
    if (adc_data) {
      server.publishAdc(adc_data.value());
    }

    if (camera.isOpened() && camera.read(frame)) {
      server.publishCameraFrame(frame);
    }

    const uint64_t remaining_us = LoopPeriodUs - (timing::getNowUs() - now);
    if (remaining_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(remaining_us));
    } else {
      std::cout << "Loop overrun by " << -remaining_us << " us" << std::endl;
    }
  }
}