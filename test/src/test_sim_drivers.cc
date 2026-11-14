#include "msensor/adc/sim_adc.hh"
#include "msensor/camera/sim_camera.hh"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace {

/// First `count` sequence numbers a fresh SimCamera produces.
std::vector<uint32_t> firstCameraSequenceNumbers(size_t count) {
  msensor::SimCamera camera;
  std::mutex m;
  std::condition_variable cv;
  std::vector<uint32_t> seqs;
  camera.setFrameCallback([&](msensor::CameraFrame frame) {
    std::lock_guard lock(m);
    if (seqs.size() < count) {
      EXPECT_EQ(frame.mat.cols, 640);
      EXPECT_EQ(frame.mat.rows, 480);
      seqs.push_back(frame.header.sequence_number);
    }
    cv.notify_all();
  });
  camera.startSampling();
  {
    std::unique_lock lock(m);
    EXPECT_TRUE(cv.wait_for(lock, 5s, [&] { return seqs.size() >= count; }));
  }
  camera.stopSampling();
  return seqs;
}

} // namespace

// Sequence numbers are per instance (they used to be a function-local static
// shared by every camera).
TEST(SimCamera, SequenceNumbersStartAtZeroForEachInstance) {
  EXPECT_EQ(firstCameraSequenceNumbers(2), (std::vector<uint32_t>{0, 1}));
  EXPECT_EQ(firstCameraSequenceNumbers(2), (std::vector<uint32_t>{0, 1}));
}

TEST(SimAdc, SequenceNumbersArePerInstanceAndUniqueAcrossThreads) {
  msensor::SimAdc first;
  msensor::SimAdc second;
  msensor::IAdc &a = first;
  msensor::IAdc &b = second;
  EXPECT_EQ(a.readSingleEnded()->header.sequence_number, 0U);
  EXPECT_EQ(b.readSingleEnded()->header.sequence_number, 0U);
  EXPECT_EQ(a.readSingleEnded()->header.sequence_number, 1U);

  // gRPC may serve concurrent getAdcData calls on one ADC.
  constexpr int kThreads = 4;
  constexpr int kReads = 2000;
  std::vector<std::vector<uint32_t>> seen(kThreads);
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t] {
      for (int i = 0; i < kReads; ++i) {
        const auto sample = b.readSingleEnded();
        ASSERT_TRUE(sample.has_value());
        EXPECT_GT(sample->header.timestamp, 0U);
        EXPECT_EQ(sample->timestamp, sample->header.timestamp);
        seen[static_cast<size_t>(t)].push_back(sample->header.sequence_number);
      }
    });
  }
  for (auto &thread : threads) {
    thread.join();
  }
  std::vector<uint32_t> all;
  for (const auto &v : seen) {
    all.insert(all.end(), v.begin(), v.end());
  }
  std::sort(all.begin(), all.end());
  ASSERT_EQ(all.size(), static_cast<size_t>(kThreads * kReads));
  for (size_t i = 0; i < all.size(); ++i) {
    ASSERT_EQ(all[i], static_cast<uint32_t>(i + 1)) << "duplicate or gap";
  }
}
