#pragma once

#include <random>

#include "msensor/hub/PollingProducer.hh"
#include "msensor/interface/ILidar.hh"

namespace msensor {

/**
 * @brief Synthetic LiDAR producing scans at 40 Hz for testing.
 */
class SimLidar : public ILidar {
public:
  /// Construct a SimLidar. If `steady` is true, every scan has the same
  /// points.
  SimLidar(bool steady = false);
  /// Initialize simulator resources.
  void init() override;
  /// Begin publishing simulated scans.
  void startSampling() override;
  /// Stop publishing simulated scans.
  void stopSampling() override;
  SensorHub<Scan3DI> &scans() override { return hub_; }

private:
  std::shared_ptr<const Scan3DI> makeScan();

  bool steady_;
  std::mt19937 gen_;
  uint32_t sequence_number_ = 0;
  SensorHub<Scan3DI> hub_;
  PollingProducer<Scan3DI> producer_;
};

} // namespace msensor
