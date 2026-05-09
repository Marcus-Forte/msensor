#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string_view>
#include <thread>

#include "file/file.hh"
#include "recorder/scan_recorder.hh"
#include "sensors_remote_client.hh"

namespace {
constexpr auto kIdleSleep = std::chrono::milliseconds(5);

std::atomic_bool g_should_stop = false;

void request_stop(int) { g_should_stop = true; }

void print_usage() {
  std::cout << "Usage: remote_recorder <host:port> [output.pbscan]"
            << std::endl;
}
} // namespace

int main(int argc, char **argv) {
  if (argc < 2 || argc > 3) {
    print_usage();
    return 1;
  }

  if (std::string_view(argv[1]) == "-h" ||
      std::string_view(argv[1]) == "--help") {
    print_usage();
    return 0;
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  const std::string remote_address = argv[1];
  const auto file = std::make_shared<msensor::File>();
  msensor::ScanRecorder recorder(file);
  bool has_reported_lidar_flow = false;
  bool has_reported_imu_flow = false;

  if (argc == 3) {
    recorder.start(argv[2]);
  } else {
    recorder.start();
  }

  SensorsRemoteClient client(remote_address);
  client.init();
  std::cout << "Connecting to " << remote_address << "..." << std::endl;
  client.start();

  while (!g_should_stop.load()) {
    bool recorded_sample = false;

    while (const auto scan = client.getScan()) {
      recorder.record(scan);
      recorded_sample = true;
      if (!has_reported_lidar_flow) {
        std::cout << "Connected to " << remote_address
                  << "; LiDAR data is flowing." << std::endl;
        has_reported_lidar_flow = true;
      }
    }

    while (const auto imu = client.getImuData()) {
      recorder.record(*imu);
      recorded_sample = true;
      if (!has_reported_imu_flow) {
        std::cout << "Connected to " << remote_address
                  << "; IMU data is flowing." << std::endl;
        has_reported_imu_flow = true;
      }
    }

    if (!recorded_sample) {
      std::this_thread::sleep_for(kIdleSleep);
    }
  }

  client.stop();
  recorder.stop();
  return 0;
}