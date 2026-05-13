#include "msensor/imu/sim_imu.hh"
#include "msensor/timing/timing.hh"
#include <random>
#include <thread>

namespace msensor {

std::optional<IMUData> SimImu::getImuData() {
  static uint32_t sequence_number = 0;
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_real_distribution<> dis(-1.0, 1.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50 Hz.
  IMUData data;
  data.header = Header{timing::getNowNs(), sequence_number++};
  data.ax = dis(gen);
  data.ay = dis(gen);
  data.az = dis(gen);
  data.gx = dis(gen);
  data.gy = dis(gen);
  data.gz = dis(gen);

  return data;
}

} // namespace msensor