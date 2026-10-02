#include "msensor/lidar/sim_lidar.hh"
#include "msensor/timing/timing.hh"
#include <iostream>

namespace msensor {

namespace {
constexpr auto kPeriod = std::chrono::milliseconds(25); // 40 Hz.
constexpr int kNumPoints = 2000;
} // namespace

SimLidar::SimLidar(bool steady)
    : steady_(steady), gen_(std::random_device{}()),
      producer_(hub_, [this] { return makeScan(); }, kPeriod) {
  std::cout << "SimLidar initialized. steady=" << std::boolalpha << steady_
            << std::endl;
}

void SimLidar::init() { std::cout << "init" << std::endl; }

void SimLidar::startSampling() {
  std::cout << "startSampling" << std::endl;
  producer_.start();
}

void SimLidar::stopSampling() {
  std::cout << "stopSampling" << std::endl;
  producer_.stop();
}

std::shared_ptr<const Scan3DI> SimLidar::makeScan() {
  auto scan = std::make_shared<Scan3DI>();
  scan->points.reserve(kNumPoints);

  std::uniform_real_distribution<> dis(-10.0, 10.0);
  if (!steady_) {
    for (int i = 0; i < kNumPoints; ++i) {
      scan->points.emplace_back(dis(gen_), dis(gen_), dis(gen_), i);
    }
  } else {
    std::mt19937 fixed(67); // fixed seed for deterministic output
    for (int i = 0; i < kNumPoints; ++i) {
      scan->points.emplace_back(dis(fixed), dis(fixed), dis(fixed), i);
    }
  }

  scan->header = Header{timing::getNowNs(), sequence_number_++};
  return scan;
}
} // namespace msensor
