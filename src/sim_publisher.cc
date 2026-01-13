#include <getopt.h>
#include <iostream>
#include <thread>

#include "adc/sim_adc.hh"
#include "camera/sim_camera.hh"
#include "imu/sim_imu.hh"
#include "lidar/sim_lidar.hh"
#include "sensors_server.hh"

void print_usage() { std::cout << "Usage: sim_publisher" << std::endl; }

int main(int argc, char **argv) {

  auto sim_lidar = std::make_shared<msensor::SimLidar>();
  auto sim_imu = std::make_shared<msensor::SimImu>();
  auto sim_camera = std::make_shared<msensor::SimCamera>();
  auto sim_adc = std::make_shared<msensor::SimAdc>();


  SensorsServer server(sim_adc, sim_camera, sim_imu, sim_lidar);
  server.start();

  std::cout << "Publishing scan and Imu data";
  while (true) {
    const auto scan = sim_lidar->getScan();
    const auto imudata = sim_imu->getImuData();

    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}