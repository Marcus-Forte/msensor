#include <exception>
#include <iostream>

#include "msensor/config/config.hh"
#include "msensor/factory/sensor_factory.hh"
#include "msensor_server.hh"
#include "shutdown_signal.hh"

int main() {
  try {
    msensor::Config config;
    config.rplidar.enable = false;
    config.camera.enable = false;
    config.sim = {.lidar = true, .imu = true, .camera = true, .adc = true};

    msensor::SensorsServer server(msensor::makeSensors(config));
    server.start();

    std::cout << "Publishing scan and Imu data" << std::endl;
    // Serve until SIGINT/SIGTERM, then shut down and return so the driver
    // destructors run (the server is destroyed first).
    msensor::waitForShutdownSignal();
    server.stop();
  } catch (const std::exception &e) {
    std::cerr << "sim_publisher: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
