#pragma once

#include <random>
#include <stop_token>
#include <thread>

#include "msensor/interface/CallbackSlot.hh"
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
  void run(std::stop_token st);
  Scan3DI makeScan();

  const bool steady_;
  std::mt19937 gen_;
  uint32_t sequence_number_ = 0;
  CallbackSlot<Scan3DI> callback_;
  std::jthread thread_; // last: joined before the members above are destroyed
};

} // namespace msensor
