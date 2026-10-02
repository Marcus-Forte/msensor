#include "msensor/imu/sim_imu.hh"
#include "msensor/timing/timing.hh"
#include <chrono>

namespace msensor {

namespace {
constexpr auto kPeriod = std::chrono::milliseconds(20); // 50 Hz.
}

SimImu::SimImu() : gen_(std::random_device{}()) {}

void SimImu::startSampling() {
  if (thread_.joinable()) {
    return;
  }
  thread_ = std::jthread([this](std::stop_token st) { run(st); });
}

void SimImu::stopSampling() {
  if (!thread_.joinable()) {
    return;
  }
  thread_.request_stop();
  thread_.join();
}

void SimImu::setImuCallback(ImuCallback callback) {
  callback_.setCallback(std::move(callback));
}

void SimImu::run(std::stop_token st) {
  while (!st.stop_requested()) {
    callback_.emit(makeSample());
    std::this_thread::sleep_for(kPeriod);
  }
}

IMUData SimImu::makeSample() {
  std::uniform_real_distribution<> dis(-1.0, 1.0);

  IMUData data;
  data.header = Header{timing::getNowNs(), sequence_number_++};
  data.ax = dis(gen_);
  data.ay = dis(gen_);
  data.az = dis(gen_);
  data.gx = dis(gen_);
  data.gy = dis(gen_);
  data.gz = dis(gen_);
  return data;
}

} // namespace msensor
