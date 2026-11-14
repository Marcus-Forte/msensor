#include "msensor/adc/sim_adc.hh"
#include "msensor/timing/timing.hh"
#include <cstdlib>

namespace msensor {

std::optional<AdcSample> SimAdc::readSingleEnded() const {
  // Simulate an ADC reading with a random voltage between 0 and 3.3V
  float voltage =
      static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 3.3f;
  const uint64_t now = timing::getNowNs();
  return AdcSample{Header{now, sequence_number_++}, voltage, now};
}

} // namespace msensor