#pragma once

#include <cstdint>
#include <optional>

#include "msensor/interface/Header.hh"

namespace msensor {

/**
 * @brief ADC measurement sample.
 */
struct AdcSample {
  Header header; ///< Acquisition time (ns) and sequence number.
  float voltage; ///< Measured voltage in volts.
};

/**
 * @brief Interface for analog-to-digital converters.
 */
class IAdc {
public:
  virtual ~IAdc() = default;

  /// Read a single-ended channel. Returns a sample with the voltage and the
  /// acquisition timestamp.
  virtual std::optional<AdcSample> readSingleEnded() const = 0;
};

} // namespace msensor
