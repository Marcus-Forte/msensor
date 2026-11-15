// Tests of SequenceGapCounter, the loss bookkeeping of SensorsRemoteClient.

#include "sequence_gap_counter.hh"

#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>

namespace {

/// Total lost over a sequence of numbers fed in order.
uint64_t lostIn(std::initializer_list<uint32_t> numbers,
                msensor::SequenceGapCounter counter = {}) {
  uint64_t lost = 0;
  for (const uint32_t n : numbers) {
    lost += counter.observe(n);
  }
  return lost;
}

} // namespace

TEST(SequenceGapCounter, ConsecutiveNumbersLoseNothing) {
  EXPECT_EQ(lostIn({0, 1, 2, 3, 4, 5}), 0U);
}

TEST(SequenceGapCounter, AForwardJumpOfNCountsNMinusOne) {
  EXPECT_EQ(lostIn({10, 11, 14}), 2U);
  EXPECT_EQ(lostIn({10, 12}), 1U);
  EXPECT_EQ(lostIn({0, 1000}), 999U);
  EXPECT_EQ(lostIn({1, 3, 7, 8}), 1U + 3U);
}

TEST(SequenceGapCounter, TheFirstNumberOfAStreamCarriesNoMeaning) {
  EXPECT_EQ(lostIn({0}), 0U);
  EXPECT_EQ(lostIn({123456}), 0U);
  EXPECT_EQ(lostIn({0xFFFFFFF0U, 0xFFFFFFF1U}), 0U);
  msensor::SequenceGapCounter counter;
  EXPECT_EQ(counter.observe(5000), 0U);
}

TEST(SequenceGapCounter, WrapAroundAtTwoToThe32IsNotALoss) {
  EXPECT_EQ(lostIn({0xFFFFFFFEU, 0xFFFFFFFFU, 0, 1}), 0U);
}

TEST(SequenceGapCounter, AGapAcrossTheWrapIsCountedModulo2To32) {
  // 0xFFFFFFFE -> 1 is a step of 3: two samples (0xFFFFFFFF and 0) missing.
  EXPECT_EQ(lostIn({0xFFFFFFFEU, 1}), 2U);
  EXPECT_EQ(lostIn({0xFFFFFFFFU, 1}), 1U);
  EXPECT_EQ(lostIn({0xFFFFFFFFU, 0}), 0U);
}

TEST(SequenceGapCounter, ABackwardJumpIsARestartNotALoss) {
  EXPECT_EQ(lostIn({100, 101, 0, 1, 2}), 0U);
  EXPECT_EQ(lostIn({100, 99}), 0U);
  // The numbering goes on from the restart point.
  EXPECT_EQ(lostIn({100, 101, 0, 1, 4}), 2U);
}

TEST(SequenceGapCounter, ARepeatedNumberIsNotALoss) {
  EXPECT_EQ(lostIn({7, 7, 7, 8}), 0U);
}

TEST(SequenceGapCounter, ResetStartsANewStream) {
  msensor::SequenceGapCounter counter;
  EXPECT_EQ(counter.observe(1), 0U);
  EXPECT_EQ(counter.observe(2), 0U);
  counter.reset();
  EXPECT_EQ(counter.observe(500), 0U); // not a jump of 498
  EXPECT_EQ(counter.observe(502), 1U);
}

TEST(SequenceGapCounter, ObserveReportsTheLossOfEachSample) {
  msensor::SequenceGapCounter counter;
  EXPECT_EQ(counter.observe(0), 0U);
  EXPECT_EQ(counter.observe(1), 0U);
  EXPECT_EQ(counter.observe(5), 3U);
  EXPECT_EQ(counter.observe(6), 0U);
}
