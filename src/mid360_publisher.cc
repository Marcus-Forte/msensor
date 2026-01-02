#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "file/file.hh"
#include "lidar/mid360.hh"
#include "recorder/scan_recorder.hh"
#include "sensors_server.hh"

void printUsage() {
  std::cout << "Usage: app [config] [accusamples] [mode: 0, 1,2,3] [0: "
               "non-record, 1:record]"
            << std::endl;
}
int main(int argc, char **argv) {
  if (argc < 5) {
    printUsage();
    exit(0);
  }

  const int accumulate = atoi(argv[2]);
  std::cout << "Accu samples: " << accumulate << std::endl;
  msensor::Mid360 lidar(argv[1], accumulate);
  lidar.init();

  const auto mode = atoi(argv[3]);

  if (mode == 0) {
    lidar.setMode(msensor::Mid360::Mode::PowerSave);
  } else {
    lidar.setMode(msensor::Mid360::Mode::Normal);
    if (mode == 1) {
      lidar.setScanPattern(msensor::Mid360::ScanPattern::NonRepetitive);
    } else if (mode == 2) {
      lidar.setScanPattern(msensor::Mid360::ScanPattern::Repetitive);
    } else {
      lidar.setScanPattern(msensor::Mid360::ScanPattern::LowFrameRate);
    }
  }

  const auto record = atoi(argv[4]);
  auto file = std::make_shared<msensor::File>();
  msensor::ScanRecorder recorder(file);

  if (record == 1) {
    recorder.start();
    std::cout << "Recording scans." << std::endl;
  }

  lidar.startSampling();

  SensorsServer server;
  server.start();

  std::chrono::high_resolution_clock::time_point last =
      std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point current;

  std::cout << std::fixed << std::setprecision(9);
  while (true) {
    const auto imu = lidar.getImuData();

    if (imu) {
      // std::cout << "Imu time: " << imu->timestamp << std::endl;
      // std::cout << imu->az << "," << imu->timestamp << std::endl;
      recorder.record(imu.value());
      server.publishImu(imu.value());
    }
    const auto cloud = lidar.getScan();

    if (cloud) {
      current = std::chrono::high_resolution_clock::now();
      std::cout << "AccScan time diff: "
                << std::chrono::duration_cast<std::chrono::microseconds>(
                       current - last)
                       .count()
                << " us" << std::endl;

      last = current;

      std::cout << "Sending: " << cloud->points->size() << " points"
                << std::endl;
      const double stamp_s =
          static_cast<double>(cloud->timestamp) / 1000000000.0;
      std::cout << "First point stamp " << stamp_s << '\n';
      std::cout << "Last point stamp:"
                << stamp_s +
                       (static_cast<double>(cloud->points->size()) / 200000.0)
                << '\n';
      recorder.record(cloud);
      server.publishScan(cloud);
    }

    /// \todo sleep just enough
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}