
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <thread>

#include "lidar/rp_lidar.hh"
#include "sensors_server.hh"

void print_usage() {
  std::cout << "Usage: rplidar_publisher [serial device path]" << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 2) {
    print_usage();
    exit(0);
  }

  if (!std::filesystem::exists(argv[1])) {
    std::cerr << "Device: " << argv[1] << " does not exist. Exiting..."
              << std::endl;
    exit(-1);
  }
  const auto lidar = std::make_shared<msensor::RPLidar>(argv[1]);

  lidar->init();
  lidar->setMotorRPM(360);

  SensorsServer server(nullptr, nullptr, nullptr, lidar);
  server.start();

  while (true) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}