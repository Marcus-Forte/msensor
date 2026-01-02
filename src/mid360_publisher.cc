#include <chrono>
#include <iostream>
#include <thread>

#include "lidar/mid360.hh"
#include "sensors_server.hh"

void printUsage() {
  std::cout << "Usage: app [config] [accusamples] [mode: 0, 1,2,3]"
            << std::endl;
}
int main(int argc, char **argv) {
  if (argc < 4) {
    printUsage();
    exit(0);
  }

  const int accumulate = atoi(argv[2]);
  std::cout << "Accu samples: " << accumulate << std::endl;
  auto lidar = std::make_shared<msensor::Mid360>(argv[1], accumulate);
  lidar->init();

  const auto mode = atoi(argv[3]);

  if (mode == 0) {
    lidar->setMode(msensor::Mid360::Mode::PowerSave);
  } else {
    lidar->setMode(msensor::Mid360::Mode::Normal);
    if (mode == 1) {
      lidar->setScanPattern(msensor::Mid360::ScanPattern::NonRepetitive);
    } else if (mode == 2) {
      lidar->setScanPattern(msensor::Mid360::ScanPattern::Repetitive);
    } else {
      lidar->setScanPattern(msensor::Mid360::ScanPattern::LowFrameRate);
    }
  }

  const auto record = atoi(argv[4]);

  lidar->startSampling();

  SensorsServer server(nullptr, nullptr, nullptr, lidar);
  server.start();

  while (true) {

    /// \todo sleep just enough
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}