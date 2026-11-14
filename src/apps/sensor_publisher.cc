#include <exception>
#include <filesystem>
#include <iostream>
#include <string_view>

#include "msensor/config/config.hh"
#include "msensor/factory/sensor_factory.hh"
#include "msensor_server.hh"
#include "shutdown_signal.hh"

static void print_usage() {
  std::cout << "Usage: sensor_publisher [config.json]" << std::endl;
}

int main(int argc, char **argv) {
  if (argc > 2) {
    print_usage();
    return 1;
  }

  if (argc == 2 && std::string_view(argv[1]) == "-h") {
    print_usage();
    return 0;
  }

  try {
    const std::filesystem::path config_path =
        argc == 2 ? std::filesystem::path(argv[1])
                  : msensor::Config::defaultConfigPath();
    std::cout << "Loading config from: " << config_path << std::endl;
    const msensor::Config config = msensor::Config::fromFile(config_path);

    msensor::SensorsServer server(
        msensor::makeSensors(config),
        {.camera_jpeg_quality = config.camera.jpeg_quality});
    server.start();
    // Serve until SIGINT/SIGTERM, then shut down and return so the driver
    // destructors run (the server is destroyed first).
    msensor::waitForShutdownSignal();
    server.stop();
  } catch (const std::exception &e) {
    std::cerr << "sensor_publisher: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
