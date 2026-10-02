#pragma once

#include <random>
#include <stop_token>
#include <thread>

#include "msensor/interface/CallbackSlot.hh"
#include "msensor/interface/IImu.hh"

namespace msensor {
/**
 * @brief IMU simulator publishing synthetic data at 50 Hz.
 */
class SimImu : public IImu {
public:
  SimImu();
  void startSampling() override;
  void stopSampling() override;
  /// Register the single sample consumer.
  void setImuCallback(ImuCallback callback) override;

private:
  void run(std::stop_token st);
  IMUData makeSample();

  std::mt19937 gen_;
  uint32_t sequence_number_ = 0;
  CallbackSlot<IMUData> callback_;
  std::jthread thread_; // last: joined before the members above are destroyed
};

} // namespace msensor
