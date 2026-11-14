#include <getopt.h>
#include <iostream>
#include <thread>

#include "msensor/adc/sim_adc.hh"
#include "msensor/camera/sim_camera.hh"
#include "msensor/imu/sim_imu.hh"
#include "msensor/lidar/sim_lidar.hh"
#include "msensor_server.hh"
#include "shutdown_signal.hh"

void print_usage() { std::cout << "Usage: sim_publisher" << std::endl; }

int main(int argc, char **argv) {

  auto sim_lidar = std::make_shared<msensor::SimLidar>(false);
  auto sim_imu = std::make_shared<msensor::SimImu>();
  auto sim_camera = std::make_shared<msensor::SimCamera>();
  auto sim_adc = std::make_shared<msensor::SimAdc>();

  sim_lidar->startSampling();
  sim_imu->startSampling();
  sim_camera->startSampling();

  SensorsServer server(sim_adc, sim_camera, sim_imu, sim_lidar);
  server.start();

  std::cout << "Publishing scan and Imu data" << std::endl;
  // Serve until SIGINT/SIGTERM, then shut down and return so the driver
  // destructors run (the server is destroyed first).
  msensor::waitForShutdownSignal();
  server.stop();
  return 0;
}