#pragma once

#include <random>
#include <vector>

#include "msensor/interface/CallbackSlot.hh"
#include "msensor/interface/SamplingThread.hh"
#include "msensor/interface/ILidar.hh"

namespace msensor {

/**
 * @brief Synthetic LiDAR producing scans at 40 Hz for testing.
 */
class SimLidar : public ILidar {
public:
  /// Construct a SimLidar. If `steady` is true, every scan has the same
  /// points.
  explicit SimLidar(bool steady = false);
  /// Initialize simulator resources.
  void init() override;
  /// Begin publishing simulated scans.
  void startSampling() override;
  /// Stop publishing simulated scans.
  void stopSampling() override;
  /// Register the single scan consumer.
  void setScanCallback(ScanCallback callback) override;

private:
  Scan3DI makeScan();
  static std::vector<Point3I> makeSteadyPoints();

  const bool steady_;
  std::mt19937 gen_;
  std::vector<Point3I> steady_points_; // filled once when steady_
  uint32_t sequence_number_ = 0;
  CallbackSlot<Scan3DI> callback_;
  SamplingThread sampler_; // last: joined before the members above are destroyed
};

} // namespace msensor
