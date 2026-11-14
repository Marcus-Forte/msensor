#include "msensor/imu/sim_imu.hh"
#include "msensor/timing/timing.hh"
#include <chrono>

namespace msensor {

namespace {
constexpr auto kPeriod = std::chrono::milliseconds(20); // 50 Hz.
}

SimImu::SimImu() : gen_(std::random_device{}()) {}

void SimImu::startSampling() {
  sampler_.startPeriodic(kPeriod, [this] {
    callback_.emit(makeSample());
  });
}

void SimImu::stopSampling() {
  sampler_.stop();
}

void SimImu::setImuCallback(ImuCallback callback) {
  callback_.setCallback(std::move(callback));
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
