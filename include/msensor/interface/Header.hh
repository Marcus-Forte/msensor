#pragma once

#include <cstdint>

namespace msensor {

struct Header {
  uint64_t timestamp;       ///< Acquisition timestamp in nanoseconds.
  uint32_t sequence_number; ///< Sequence number of the data sample, incremented
                            ///< for each new sample.
};
} // namespace msensor