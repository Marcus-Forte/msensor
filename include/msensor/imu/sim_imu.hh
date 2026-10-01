#pragma once

#include <random>

#include "msensor/hub/PollingProducer.hh"
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
  SensorHub<IMUData> &imu() override { return hub_; }

private:
  std::shared_ptr<const IMUData> makeSample();

  std::mt19937 gen_;
  uint32_t sequence_number_ = 0;
  SensorHub<IMUData> hub_;
  PollingProducer<IMUData> producer_;
};

} // namespace msensor
