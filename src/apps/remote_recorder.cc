#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string_view>
#include <thread>

#include "msensor/file/file.hh"
#include "msensor/recorder/scan_recorder.hh"
#include "sensors_remote_client.hh"

namespace {
constexpr auto kIdleSleep = std::chrono::milliseconds(100);

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
  std::atomic<bool> lidar_flow{false};
  std::atomic<bool> imu_flow{false};
  std::atomic<std::size_t> lidar_entries_saved{0};
  std::atomic<std::size_t> imu_entries_saved{0};

  if (argc == 3) {
    std::string output_filename = argv[2];
    if (!output_filename.ends_with(".pbscan")) {
      output_filename += ".pbscan";
    }
    recorder.start(output_filename);
  } else {
    recorder.start();
  }

  SensorsRemoteClient client(remote_address);
  client.init();
  std::cout << "Connecting to " << remote_address << "..." << std::endl;

  // The callbacks run on the reader threads; the recorder is thread-safe.
  client.setScanCallback([&](const msensor::Scan3DI &scan) {
    recorder.record(scan);
    ++lidar_entries_saved;
    if (!lidar_flow.exchange(true)) {
      std::cout << "Connected to " << remote_address << "; Receiving LiDAR data"
                << std::endl;
    }
  });
  client.setImuCallback([&](const msensor::IMUData &imu) {
    recorder.record(imu);
    ++imu_entries_saved;
    if (!imu_flow.exchange(true)) {
      std::cout << "Connected to " << remote_address << "; Receiving IMU data"
                << std::endl;
    }
  });
  client.start();

  while (!g_should_stop.load()) {
    std::this_thread::sleep_for(kIdleSleep);
  }

  client.stop();
  recorder.stop();
  std::cout << "Saved recording to " << recorder.getFilename() << std::endl;
  std::cout << "Saved entries - LiDAR: " << lidar_entries_saved
            << ", IMU: " << imu_entries_saved << std::endl;
  return 0;
}
