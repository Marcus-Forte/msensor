#include <getopt.h>
#include <iostream>

#include "file/file.hh"
#include "imu/sim_imu.hh"
#include "lidar/sim_lidar.hh"
#include "recorder/scan_recorder.hh"
#include "sensors_server.hh"

void print_usage() {
  std::cout << "Usage: sim_publisher [-r record]" << std::endl;
}

int main(int argc, char **argv) {

  auto simLidar = std::make_unique<msensor::SimLidar>();
  auto simImu = std::make_unique<msensor::SimImu>();

  auto file = std::make_shared<msensor::File>();
  msensor::ScanRecorder recorder(file);

  bool record_scans = false;
  int opt;
  while ((opt = getopt(argc, argv, "rh")) != -1) {
    switch (opt) {
    case 'h':
      print_usage();
      exit(0);
    case 'r':
      record_scans = true;
      break;
    }
  }

  if (record_scans) {
    std::cout << "Recording scan enabled" << std::endl;
    recorder.start();
  }

  SensorsServer server;
  server.start();

  std::cout << "Publishing scan and Imu data";
  while (true) {
    const auto scan = simLidar->getScan();
    const auto imudata = simImu->getImuData();

    server.publishScan(scan);
    recorder.record(scan);
    if (imudata) {
      server.publishImu(imudata.value());
      recorder.record(imudata.value());
    }
  }

  server.stop();
}