#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "msensor/bus/sensor_bus.hh"
#include "msensor/config/config.hh"
#include "msensor/factory/sensor_factory.hh"
#include "msensor/recorder/bus_recorder.hh"
#include "msensor_server.hh"
#include "shutdown_signal.hh"

static void print_usage() {
  std::cout << "Usage: sensor_publisher [config.json] [--record <file.pbscan>]"
            << std::endl;
}

int main(int argc, char **argv) {
  std::optional<std::filesystem::path> config_arg;
  std::optional<std::string> record_file;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_usage();
      return 0;
    }
    if (arg == "--record") {
      if (i + 1 >= argc || record_file) {
        print_usage();
        return 1;
      }
      record_file = argv[++i];
    } else if (arg.starts_with("--") || config_arg) {
      print_usage();
      return 1;
    } else {
      config_arg = argv[i];
    }
  }

  try {
    const std::filesystem::path config_path =
        config_arg ? *config_arg : msensor::Config::defaultConfigPath();
    std::cout << "Loading config from: " << config_path << std::endl;
    const msensor::Config config = msensor::Config::fromFile(config_path);

    // One bus, shared by the gRPC server and the recorder. Declared first, so
    // it (and the drivers it owns) is destroyed last.
    auto bus =
        std::make_shared<msensor::SensorBus>(msensor::makeSensors(config));

    std::unique_ptr<msensor::BusRecorder> recorder;
    if (record_file || config.recording.enable) {
      recorder = std::make_unique<msensor::BusRecorder>(
          bus,
          msensor::BusRecorderOptions{.scan_queue = config.recording.scan_queue,
                                      .imu_queue = config.recording.imu_queue});
      recorder->start(record_file ? *record_file : config.recording.file);
      std::cout << "Recording to " << recorder->filename() << std::endl;
    }

    msensor::SensorsServer server(
        {.camera_jpeg_quality = config.camera.jpeg_quality}, bus);
    server.start();
    // Serve until SIGINT/SIGTERM, then stop the server, drain and close the
    // recording, and return so the driver destructors run (after the
    // recorder, which unsubscribes from the bus first).
    msensor::waitForShutdownSignal();
    server.stop();
    if (recorder) {
      const msensor::BusRecorderStats stats = recorder->stop();
      std::cout << "Saved recording to " << recorder->filename()
                << "; entries - LiDAR: " << stats.lidar.written
                << ", IMU: " << stats.imu.written
                << "; dropped - LiDAR: " << stats.lidar.dropped
                << ", IMU: " << stats.imu.dropped << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "sensor_publisher: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
