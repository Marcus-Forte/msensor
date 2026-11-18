#include <atomic>
#include <chrono>
#include <csignal>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "msensor/file/file.hh"
#include "msensor/recorder/scan_recorder.hh"
#include "sensors_remote_client.hh"

namespace {
constexpr auto kIdleSleep = std::chrono::milliseconds(100);

std::atomic_bool g_should_stop = false;

void request_stop(int) { g_should_stop = true; }

void print_usage(const char *program) {
  std::cout
      << "Usage: " << program << " -a <host:port> [-o <output.pbscan>]\n"
      << "  -a, --address <host:port>  Sensor server to record (required)\n"
      << "  -o, --output <file>        Recording to write [default: "
         "scan_<time>.pbscan]\n"
      << "  -h, --help                 Show this help message\n"
      << "Ctrl-C stops the recording and closes the file.\n";
}

} // namespace

int main(int argc, char **argv) {
  std::string address;
  std::string output;

  static const option long_options[] = {
      {"address", required_argument, nullptr, 'a'},
      {"output", required_argument, nullptr, 'o'},
      {"help", no_argument, nullptr, 'h'},
      {nullptr, 0, nullptr, 0}};

  int opt;
  while ((opt = getopt_long(argc, argv, "a:o:h", long_options, nullptr)) !=
         -1) {
    switch (opt) {
    case 'a':
      address = optarg;
      break;
    case 'o':
      output = optarg;
      break;
    case 'h':
      print_usage(argv[0]);
      return 0;
    default:
      print_usage(argv[0]);
      return 1;
    }
  }

  if (optind < argc) {
    std::cerr << "Error: unexpected argument: " << argv[optind] << "\n";
    print_usage(argv[0]);
    return 1;
  }
  if (address.empty()) {
    std::cerr << "Error: a sensor server address is required.\n";
    print_usage(argv[0]);
    return 1;
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  const auto file = std::make_shared<msensor::File>();
  msensor::ScanRecorder recorder(file);
  std::atomic<bool> lidar_flow{false};
  std::atomic<bool> imu_flow{false};
  std::atomic<std::size_t> lidar_entries_saved{0};
  std::atomic<std::size_t> imu_entries_saved{0};

  if (!output.empty()) {
    if (!output.ends_with(".pbscan")) {
      output += ".pbscan";
    }
    recorder.start(output);
  } else {
    recorder.start();
  }

  // A recorder wants every sample: ~5 s of IMU at 200 Hz and 16 scans (about
  // 2.4 MiB of Mid360 scans) of slack for a slow disk.
  msensor::SensorsRemoteClient client(
      address, {.scan_queue_size = 16, .imu_queue_size = 1000});
  client.init();
  std::cout << "Connecting to " << address << "..." << std::endl;

  // The callbacks run on the reader threads; the recorder is thread-safe.
  client.setScanCallback([&](const msensor::Scan3DI &scan) {
    recorder.record(scan);
    ++lidar_entries_saved;
    if (!lidar_flow.exchange(true)) {
      std::cout << "Connected to " << address << "; Receiving LiDAR data"
                << std::endl;
    }
  });
  client.setImuCallback([&](const msensor::IMUData &imu) {
    recorder.record(imu);
    ++imu_entries_saved;
    if (!imu_flow.exchange(true)) {
      std::cout << "Connected to " << address << "; Receiving IMU data"
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
  std::cout << "Lost entries - LiDAR: " << client.lostScans()
            << ", IMU: " << client.lostImuSamples() << std::endl;
  return 0;
}
