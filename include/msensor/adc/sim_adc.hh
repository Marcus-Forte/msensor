#pragma once

#include <atomic>
#include <cstdint>

#include "msensor/interface/IAdc.hh"

namespace msensor {
class SimAdc : public IAdc {
private:
  std::optional<AdcSample> readSingleEnded() const override;

  // readSingleEnded() is const and may be called from several gRPC threads.
  mutable std::atomic<uint32_t> sequence_number_{0};
};

} // namespace msensor