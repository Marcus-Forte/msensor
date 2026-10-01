#include "msensor/imu/sim_imu.hh"
#include "msensor/timing/timing.hh"

namespace msensor {

namespace {
constexpr auto kPeriod = std::chrono::milliseconds(20); // 50 Hz.
}

SimImu::SimImu()
    : gen_(std::random_device{}()),
      producer_(hub_, [this] { return makeSample(); }, kPeriod) {}

void SimImu::startSampling() { producer_.start(); }
void SimImu::stopSampling() { producer_.stop(); }

std::shared_ptr<const IMUData> SimImu::makeSample() {
  std::uniform_real_distribution<> dis(-1.0, 1.0);

  auto data = std::make_shared<IMUData>();
  data->header = Header{timing::getNowNs(), sequence_number_++};
  data->ax = dis(gen_);
  data->ay = dis(gen_);
  data->az = dis(gen_);
  data->gx = dis(gen_);
  data->gy = dis(gen_);
  data->gz = dis(gen_);
  return data;
}

} // namespace msensor
