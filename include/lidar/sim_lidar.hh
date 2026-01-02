#pragma once

#include "interface/ILidar.hh"

namespace msensor {

/**
 * @brief Synthetic LiDAR producing deterministic scans for testing.
 */
class SimLidar : public ILidar {
public:
  /// Initialize simulator resources.
  void init() override;
  /// Begin generating simulated scans.
  void startSampling() override;
  /// Stop generating simulated scans.
  void stopSampling() override;
  /// Return the latest simulated scan.
  std::shared_ptr<Scan3DI> getScan() override;
};

} // namespace msensor