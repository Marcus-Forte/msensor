#include "msensor/adc/sim_adc.hh"
#include "msensor/camera/sim_camera.hh"
#include "msensor/lidar/sim_lidar.hh"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
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

/// Starts `driver` with no consumer, waits several periods, installs a
/// callback via `setCallback` and returns the first `count` samples delivered
/// (as `value(sample)`).
template <class Driver, class Sample, class SetCb, class Value>
auto collectAfterIdle(Driver &driver, size_t count, SetCb setCallback,
                      Value value) {
  std::mutex m;
  std::condition_variable cv;
  std::vector<decltype(value(std::declval<const Sample &>()))> out;
  driver.startSampling();
  std::this_thread::sleep_for(300ms); // many periods with no consumer
  setCallback([&](Sample sample) {
    std::lock_guard lock(m);
    if (out.size() < count) {
      out.push_back(value(sample));
    }
    cv.notify_all();
  });
  {
    std::unique_lock lock(m);
    EXPECT_TRUE(cv.wait_for(lock, 30s, [&] { return out.size() >= count; }));
  }
  setCallback({});
  driver.stopSampling();
  return out;
}

} // namespace

TEST(SimCamera, NoFramesProducedWithoutConsumer) {
  msensor::SimCamera camera;
  const auto seqs = collectAfterIdle<msensor::SimCamera, msensor::CameraFrame>(
      camera, 3,
      [&](msensor::SimCamera::FrameCallback cb) {
        camera.setFrameCallback(std::move(cb));
      },
      [](const msensor::CameraFrame &f) { return f.header.sequence_number; });
  EXPECT_EQ(seqs, (std::vector<uint32_t>{0, 1, 2}));
}

TEST(SimCamera, FramesAreIndependentCopies) {
  msensor::SimCamera camera;
  std::mutex m;
  std::condition_variable cv;
  std::vector<msensor::CameraFrame> frames;
  camera.setFrameCallback([&](msensor::CameraFrame f) {
    std::lock_guard lock(m);
    if (frames.size() < 2) {
      frames.push_back(std::move(f));
    }
    cv.notify_all();
  });
  camera.startSampling();
  {
    std::unique_lock lock(m);
    ASSERT_TRUE(cv.wait_for(lock, 30s, [&] { return frames.size() >= 2; }));
  }
  camera.setFrameCallback({});
  camera.stopSampling();
  EXPECT_NE(frames[0].mat.data, frames[1].mat.data);
  EXPECT_EQ(cv::norm(frames[0].mat, frames[1].mat, cv::NORM_INF), 0.0);
}

TEST(SimLidar, NoScansProducedWithoutConsumer) {
  msensor::SimLidar lidar(false);
  const auto seqs = collectAfterIdle<msensor::SimLidar, msensor::Scan3DI>(
      lidar, 3,
      [&](msensor::ILidar::ScanCallback cb) {
        lidar.setScanCallback(std::move(cb));
      },
      [](const msensor::Scan3DI &s) { return s.header.sequence_number; });
  EXPECT_EQ(seqs, (std::vector<uint32_t>{0, 1, 2}));
}

TEST(SimLidar, SteadyScansAreIdentical) {
  msensor::SimLidar lidar(true);
  const auto scans = collectAfterIdle<msensor::SimLidar, msensor::Scan3DI>(
      lidar, 3,
      [&](msensor::ILidar::ScanCallback cb) {
        lidar.setScanCallback(std::move(cb));
      },
      [](const msensor::Scan3DI &s) { return s; });
  ASSERT_EQ(scans.size(), 3U);
  EXPECT_EQ(scans[0].header.sequence_number, 0U);
  EXPECT_EQ(scans[2].header.sequence_number, 2U);
  ASSERT_FALSE(scans[0].points.empty());
  for (size_t i = 1; i < scans.size(); ++i) {
    ASSERT_EQ(scans[i].points.size(), scans[0].points.size());
    EXPECT_EQ(std::memcmp(scans[i].points.data(), scans[0].points.data(),
                          scans[0].points.size() * sizeof(msensor::Point3I)),
              0);
  }
}

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
