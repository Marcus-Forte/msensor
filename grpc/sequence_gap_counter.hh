#pragma once

#include <cstdint>

namespace msensor {

/**
 * @brief Counts the samples missing from a stream of `header.sequence_number`.
 *
 * Feed it the sequence number of every received sample of one stream, in
 * arrival order. A forward jump of n > 1 (modulo 2^32, so 0xFFFFFFFF -> 0 is
 * a step of 1) means n - 1 samples were lost. A jump back is a restart of the
 * source (playback rewind, server restart), not a loss, and so is a repeated
 * number. The first number after construction or reset() carries no meaning:
 * a stream may start anywhere. Jumps of 2^31 or more cannot be told from a
 * restart and count as one.
 *
 * Not thread-safe: one stream, one reader thread.
 */
class SequenceGapCounter {
public:
  /// Forget the previous number: the next sample starts a new stream (call it
  /// on every reconnect).
  void reset() noexcept { have_last_ = false; }

  /// Returns how many samples were lost between the previous sample and this
  /// one.
  uint32_t observe(uint32_t sequence_number) noexcept {
    const bool had_last = have_last_;
    const uint32_t step = sequence_number - last_; // modulo 2^32
    last_ = sequence_number;
    have_last_ = true;
    if (!had_last || step < 2 || step >= kHalfRange) {
      return 0;
    }
    return step - 1;
  }

private:
  static constexpr uint32_t kHalfRange = 0x80000000U;

  uint32_t last_ = 0;
  bool have_last_ = false;
};

} // namespace msensor
