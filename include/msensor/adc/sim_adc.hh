#pragma once

#include "msensor/interface/IAdc.hh"

namespace msensor {
class SimAdc : public IAdc {
private:
  std::optional<AdcSample> readSingleEnded() const override;
};

} // namespace msensor