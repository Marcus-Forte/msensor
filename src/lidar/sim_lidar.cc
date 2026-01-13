#include "lidar/sim_lidar.hh"
#include "timing/timing.hh"
#include <iostream>
#include <random>
#include <thread>

namespace msensor {

void SimLidar::init() { std::cout << "init" << std::endl; }

void SimLidar::startSampling() { std::cout << "startSampling" << std::endl; }
void SimLidar::stopSampling() { std::cout << "stopSampling" << std::endl; }

std::shared_ptr<Scan3DI> SimLidar::getScan() {

  const int nr_points = 100;

  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_real_distribution<> dis(-1.0, 1.0);

  auto scan = std::make_shared<Scan3DI>();
  scan->points->reserve(nr_points);

  for (int i = 0; i < nr_points; ++i) {
    scan->points->emplace_back(dis(gen), dis(gen), dis(gen), i % nr_points);
  }

  scan->timestamp = timing::getNowUs();

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz.

  return scan;
}
} // namespace msensor