#pragma once

#include "interface/IImu.hh"

namespace msensor {
/**
 * @brief IMU simulator returning synthetic data.
 */
class SimImu : public IImu {
public:
  /// Produce a synthetic IMU sample.
  std::optional<IMUData> getImuData() override;
};

} // namespace msensor