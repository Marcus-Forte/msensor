#include "lidar/sim_lidar.hh"
#include "timing/timing.hh"
#include <iostream>
#include <random>
#include <thread>

namespace msensor {

SimLidar::SimLidar(bool steady) : steady_(steady) {
  std::cout << "SimLidar initialized. steady=" << std::boolalpha << steady_ << std::endl;
}

void SimLidar::init() { std::cout << "init" << std::endl; }

void SimLidar::startSampling() { std::cout << "startSampling" << std::endl; }
void SimLidar::stopSampling() { std::cout << "stopSampling" << std::endl; }

std::shared_ptr<Scan3DI> SimLidar::getScan() {

  const int nr_points = 2000;

  std::random_device rd;
  std::mt19937 gen(rd());

  auto scan = std::make_shared<Scan3DI>();
  scan->points->reserve(nr_points);

  if (!steady_) {
    std::uniform_real_distribution<> dis(-10.0, 10.0);
    for (int i = 0; i < nr_points; ++i) {
      scan->points->emplace_back(dis(gen), dis(gen), dis(gen), i % nr_points);
    }
  } else {
    std::mt19937 gen(67); // fixed seed for deterministic output
    std::uniform_real_distribution<> dis(-10.0, 10.0);
    for (int i = 0; i < nr_points; ++i) {
      scan->points->emplace_back(dis(gen), dis(gen), dis(gen), i % nr_points);
    }
  }

  scan->timestamp = timing::getNowUs();

  std::this_thread::sleep_for(std::chrono::milliseconds(25)); // 40 Hz.

  return scan;
}
} // namespace msensor