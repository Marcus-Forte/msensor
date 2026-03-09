#pragma once

#include "interface/ILidar.hh"

namespace msensor {

/**
 * @brief Synthetic LiDAR producing deterministic scans for testing.
 */
class SimLidar : public ILidar {
public:
  /// Construct a SimLidar. If `steady` is true, the same scan will be returned on each call to `getScan()`.  
  SimLidar(bool steady = false);
  /// Initialize simulator resources.
  void init() override;
  /// Begin generating simulated scans.
  void startSampling() override;
  /// Stop generating simulated scans.
  void stopSampling() override;
  /// Return the latest simulated scan.
  std::shared_ptr<Scan3DI> getScan() override;

  private:
  bool steady_;
};

} // namespace msensor