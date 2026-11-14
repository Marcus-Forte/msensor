#include "msensor/file/file.hh"
#include "msensor/recorder/recording_driver.hh"
#include "msensor/recorder/scan_recorder.hh"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <variant>
#include <vector>

using namespace std::chrono_literals;

namespace {

using Entry = std::variant<msensor::Scan3DI, msensor::IMUData>;

msensor::Scan3DI makeScan(uint32_t seq, size_t points) {
  msensor::Scan3DI scan;
  scan.header = {5'000'000ULL * seq + 1, seq};
  for (size_t i = 0; i < points; ++i) {
    const float f = static_cast<float>(seq) + static_cast<float>(i) * 0.001F;
    scan.points.push_back({f, -f, 2 * f, static_cast<float>(i % 200)});
  }
  return scan;
}

msensor::IMUData makeImu(uint32_t seq) {
  const float f = static_cast<float>(seq);
  return msensor::IMUData{
      {1'000'000ULL * seq + 2, seq}, f, f + 1, f + 2, -f, -f - 1, -f - 2};
}

/// A recording on disk, written through ScanRecorder, plus what it holds.
struct RecordingFixture {
  std::filesystem::path path;
  std::vector<Entry> entries;

  explicit RecordingFixture(const std::string &name) {
    path = std::filesystem::temp_directory_path() /
           ("msensor_driver_" + std::to_string(::getpid()) + "_" + name +
            ".pbscan");
    msensor::ScanRecorder recorder(std::make_shared<msensor::File>());
    recorder.start(path.string());
    uint32_t scan_seq = 0;
    for (uint32_t i = 0; i < 40; ++i) {
      if (i % 4 == 0) {
        auto scan = makeScan(scan_seq++, 300 + i);
        recorder.record(scan);
        entries.emplace_back(std::move(scan));
      }
      auto imu = makeImu(i);
      recorder.record(imu);
      entries.emplace_back(imu);
    }
  }
  ~RecordingFixture() { std::filesystem::remove(path); }
};

bool same(const Entry &a, const Entry &b) {
  if (a.index() != b.index()) {
    return false;
  }
  if (const auto *sa = std::get_if<msensor::Scan3DI>(&a)) {
    const auto &sb = std::get<msensor::Scan3DI>(b);
    return sa->header.timestamp == sb.header.timestamp &&
           sa->header.sequence_number == sb.header.sequence_number &&
           sa->points.size() == sb.points.size() &&
           (sa->points.empty() ||
            std::memcmp(sa->points.data(), sb.points.data(),
                        sa->points.size() * sizeof(msensor::Point3I)) == 0);
  }
  const auto &ia = std::get<msensor::IMUData>(a);
  const auto &ib = std::get<msensor::IMUData>(b);
  return ia.header.timestamp == ib.header.timestamp &&
         ia.header.sequence_number == ib.header.sequence_number &&
         std::memcmp(&ia.ax, &ib.ax, 6 * sizeof(float)) == 0;
}

/// Collects everything a driver emits, in emission order.
struct Sink {
  std::mutex m;
  std::condition_variable cv;
  std::vector<Entry> received;

  void attach(msensor::RecordingSensorDriver &driver) {
    driver.setScanCallback([this](msensor::Scan3DI scan) {
      std::lock_guard lock(m);
      received.emplace_back(std::move(scan));
      cv.notify_all();
    });
    driver.setImuCallback([this](msensor::IMUData imu) {
      std::lock_guard lock(m);
      received.emplace_back(imu);
      cv.notify_all();
    });
  }

  bool waitFor(size_t count, std::chrono::milliseconds timeout = 5s) {
    std::unique_lock lock(m);
    return cv.wait_for(lock, timeout, [&] { return received.size() >= count; });
  }
};

} // namespace

TEST(RecordingSensorDriver, UnpacedPlaybackEmitsEveryEntryInOrder) {
  RecordingFixture recording("in_order");
  msensor::RecordingSensorDriver driver(recording.path, 0.0);
  Sink sink;
  sink.attach(driver);
  driver.startSampling();
  ASSERT_TRUE(sink.waitFor(recording.entries.size()));
  for (int i = 0; i < 200 && !driver.isFinished(); ++i) {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(driver.isFinished());
  driver.stopSampling();

  std::lock_guard lock(sink.m);
  ASSERT_EQ(sink.received.size(), recording.entries.size());
  for (size_t i = 0; i < recording.entries.size(); ++i) {
    EXPECT_TRUE(same(sink.received[i], recording.entries[i])) << "entry " << i;
  }
}

TEST(RecordingSensorDriver, ResetReplaysFromTheStart) {
  RecordingFixture recording("reset");
  msensor::RecordingSensorDriver driver(recording.path, 0.0);
  Sink sink;
  sink.attach(driver);
  driver.startSampling();
  const size_t n = recording.entries.size();
  ASSERT_TRUE(sink.waitFor(n));
  for (int i = 0; i < 200 && !driver.isFinished(); ++i) {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(driver.isFinished());

  driver.requestReset(); // finished playback pauses; reset rewinds
  driver.setPaused(false);
  ASSERT_TRUE(sink.waitFor(2 * n));
  driver.stopSampling();
  std::lock_guard lock(sink.m);
  for (size_t i = 0; i < n; ++i) {
    EXPECT_TRUE(same(sink.received[n + i], recording.entries[i])) << i;
  }
}

// Clearing a callback must wait for an invocation that is already running:
// the gRPC reactor deletes itself right after clearing its callback.
TEST(RecordingSensorDriver, ClearingCallbackWaitsForRunningCallback) {
  RecordingFixture recording("clear");
  msensor::RecordingSensorDriver driver(recording.path, 0.0);
  std::atomic<bool> in_callback{false};
  std::atomic<bool> callback_done{false};
  std::atomic<int> calls{0};
  driver.setScanCallback([&](msensor::Scan3DI) {
    if (calls.fetch_add(1) == 0) {
      in_callback.store(true);
      std::this_thread::sleep_for(100ms);
      callback_done.store(true);
    }
  });
  driver.startSampling();
  while (!in_callback.load()) {
    std::this_thread::yield();
  }
  driver.setScanCallback({});
  EXPECT_TRUE(callback_done.load());
  const int calls_after_clear = calls.load();
  for (int i = 0; i < 200 && !driver.isFinished(); ++i) {
    std::this_thread::sleep_for(5ms);
  }
  driver.stopSampling();
  EXPECT_EQ(calls.load(), calls_after_clear);
}

TEST(RecordingSensorDriver, RejectsNegativeSpeed) {
  RecordingFixture recording("speed");
  EXPECT_THROW(msensor::RecordingSensorDriver(recording.path, -1.0),
               std::invalid_argument);
}
