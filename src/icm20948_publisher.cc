#include <getopt.h>
#include <iostream>
#include <thread>

#include "imu/icm-20948.h"
#include "imu/icm-20948_defs.h"
#include "sensors_server.hh"
#include "timing/timing.hh"

constexpr uint64_t sample_period_us = 10000; // 100 hz

int SetRealTimePriority() {
  struct sched_param sch;
  sch.sched_priority = 10;
  return sched_setscheduler(0, SCHED_FIFO, &sch);
}

void print_usage() {
  std::cout << "Usage: imu_publisher [i2c device] " << std::endl;
}

int main(int argc, char **argv) {

  SetRealTimePriority();

  if (argc < 2) {
    print_usage();
    exit(0);
  }

  auto i2c_device = std::atoi(argv[1]);

  auto icm20948 = std::make_shared<msensor::ICM20948>(i2c_device, ICM20948_ADDR0);
  icm20948->init();

  icm20948->calibrate();

  SensorsServer server(nullptr, nullptr, icm20948, nullptr);
  server.start();

  while (true) {
    std::this_thread::sleep_for(std::chrono::microseconds(sample_period_us));
  }
}
