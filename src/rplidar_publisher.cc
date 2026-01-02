
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <thread>

#include "file/file.hh"
#include "lidar/rp_lidar.hh"
#include "recorder/scan_recorder.hh"
#include "sensors_server.hh"

void print_usage() {
  std::cout << "Usage: rplidar_publisher [serial device path]  [-r record] "
            << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 2) {
    print_usage();
    exit(0);
  }

  std::shared_ptr<msensor::ILidar> lidar;
  if (!std::filesystem::exists(argv[1])) {
    std::cerr << "Device: " << argv[1] << " does not exist. Exiting..."
              << std::endl;
    exit(-1);
  }
  lidar = std::make_shared<msensor::RPLidar>(argv[1]);
  dynamic_cast<msensor::RPLidar *>(lidar.get())->setMotorRPM(360);

  auto file = std::make_shared<msensor::File>();
  msensor::ScanRecorder recorder(file);

  bool record_scans = false;
  int opt;
  while ((opt = getopt(argc, argv, "r")) != -1) {
    switch (opt) {
    case 'r':
      record_scans = true;
      std::cout << "Recording scan enabled" << std::endl;
      recorder.start();
      break;
    }
  }

  lidar->init();

  SensorsServer server(nullptr, nullptr, nullptr, lidar);
  server.start();

  while (true) {

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

}