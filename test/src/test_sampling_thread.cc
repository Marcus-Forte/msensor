#include "msensor/interface/SamplingThread.hh"

#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <mutex>
#include <thread>
#include <vector>

using namespace msensor;
using namespace std::chrono_literals;
using Clock = SamplingThread::Clock;

namespace {
// Polls with a generous deadline; returns whether `pred` became true.
template <class Pred> bool waitFor(Pred pred, std::chrono::seconds limit = 20s) {
  const auto deadline = Clock::now() + limit;
  while (!pred()) {
    if (Clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }
  return true;
}
} // namespace

TEST(SamplingThreadTest, PeriodicRunsAtNominalRateAndNeverFaster) {
  constexpr auto kPeriod = 10ms;
  std::atomic<int> ticks{0};
  SamplingThread sampler;
  const auto start = Clock::now();
  sampler.startPeriodic(kPeriod, [&] { ++ticks; });
  // Wait for a number of ticks rather than sleeping a fixed time.
  ASSERT_TRUE(waitFor([&] { return ticks >= 20; }));
  sampler.stop();
  const auto elapsed = Clock::now() - start;
  const int n = ticks;
  // The first tick is immediate and tick k is due at k * period, so n ticks
  // need at least (n - 1) periods. Exact bound, independent of machine load.
  EXPECT_LE(n - 1, elapsed / kPeriod);
  // On a loaded machine ticks may be skipped, but the rate cannot collapse.
  EXPECT_GE(n, 20);
}

TEST(SamplingThreadTest, SlowTickDoesNotCatchUpInABurst) {
  constexpr auto kPeriod = 10ms;
  std::mutex m;
  std::vector<Clock::time_point> times;
  SamplingThread sampler;
  sampler.startPeriodic(kPeriod, [&] {
    std::lock_guard lock(m);
    times.push_back(Clock::now());
    if (times.size() == 3) {
      // Overrun about eight slots; the following ticks must not be bunched.
      std::this_thread::sleep_for(8 * kPeriod);
    }
  });
  ASSERT_TRUE(waitFor([&] {
    std::lock_guard lock(m);
    return times.size() >= 8;
  }));
  sampler.stop();
  std::lock_guard lock(m);
  ASSERT_GE(times.size(), 8U);
  // After the slow tick, consecutive ticks are separated by a good part of a
  // period (a catch-up burst would fire them back to back). Allow generous
  // slack for timer granularity: at least half a period.
  for (size_t i = 4; i < times.size(); ++i) {
    EXPECT_GE(times[i] - times[i - 1], kPeriod / 2) << "tick " << i;
  }
}

TEST(SamplingThreadTest, StopIsPromptWithLongPeriod) {
  std::atomic<int> ticks{0};
  SamplingThread sampler;
  sampler.startPeriodic(1h, [&] { ++ticks; });
  ASSERT_TRUE(waitFor([&] { return ticks >= 1; }));
  const auto t0 = Clock::now();
  sampler.stop();
  EXPECT_LT(Clock::now() - t0, 1s);
  EXPECT_FALSE(sampler.running());
}

TEST(SamplingThreadTest, StopIsPromptWithLongBackoff) {
  std::atomic<int> steps{0};
  SamplingThread sampler;
  sampler.startFreeRunning([&]() -> std::chrono::nanoseconds {
    ++steps;
    return 1h;
  });
  ASSERT_TRUE(waitFor([&] { return steps >= 1; }));
  const auto t0 = Clock::now();
  sampler.stop();
  EXPECT_LT(Clock::now() - t0, 1s);
  EXPECT_EQ(steps, 1);
}

TEST(SamplingThreadTest, FreeRunningWithoutBackoffRepeats) {
  std::atomic<int> steps{0};
  SamplingThread sampler;
  sampler.startFreeRunning([&]() -> std::chrono::nanoseconds {
    ++steps;
    return {};
  });
  EXPECT_TRUE(waitFor([&] { return steps >= 1000; }));
}

TEST(SamplingThreadTest, FreeRunningHonoursBackoff) {
  constexpr auto kBackoff = 5ms;
  std::atomic<int> steps{0};
  SamplingThread sampler;
  const auto start = Clock::now();
  sampler.startFreeRunning([&]() -> std::chrono::nanoseconds {
    ++steps;
    return kBackoff;
  });
  ASSERT_TRUE(waitFor([&] { return steps >= 10; }));
  sampler.stop();
  // n steps are separated by n - 1 back-offs.
  EXPECT_LE(steps - 1, (Clock::now() - start) / kBackoff);
}

TEST(SamplingThreadTest, DoubleStartIsANoOp) {
  std::atomic<int> first{0};
  std::atomic<int> second{0};
  SamplingThread sampler;
  sampler.startPeriodic(1ms, [&] { ++first; });
  EXPECT_TRUE(sampler.running());
  sampler.startPeriodic(1ms, [&] { ++second; });
  sampler.startFreeRunning([&]() -> std::chrono::nanoseconds {
    ++second;
    return {};
  });
  ASSERT_TRUE(waitFor([&] { return first >= 5; }));
  sampler.stop();
  EXPECT_EQ(second, 0);
}

TEST(SamplingThreadTest, StopWhenNotRunningIsANoOp) {
  SamplingThread sampler;
  EXPECT_FALSE(sampler.running());
  sampler.stop();
  sampler.stop();
}

TEST(SamplingThreadTest, RestartAfterStopWorks) {
  SamplingThread sampler;
  std::atomic<int> a{0};
  sampler.startPeriodic(1ms, [&] { ++a; });
  ASSERT_TRUE(waitFor([&] { return a >= 3; }));
  sampler.stop();
  const int a_final = a;
  std::atomic<int> b{0};
  sampler.startPeriodic(1ms, [&] { ++b; });
  EXPECT_TRUE(sampler.running());
  ASSERT_TRUE(waitFor([&] { return b >= 3; }));
  sampler.stop();
  EXPECT_EQ(a, a_final); // the first loop is really gone
  // Also switch mode on restart.
  std::atomic<int> c{0};
  sampler.startFreeRunning([&]() -> std::chrono::nanoseconds {
    ++c;
    return 1ms;
  });
  EXPECT_TRUE(waitFor([&] { return c >= 3; }));
}

TEST(SamplingThreadTest, DestructionWhileRunningJoins) {
  std::atomic<int> ticks{0};
  {
    SamplingThread sampler;
    sampler.startPeriodic(1h, [&] { ++ticks; });
    ASSERT_TRUE(waitFor([&] { return ticks >= 1; }));
  } // must return promptly and join
  const int after = ticks;
  std::this_thread::sleep_for(5ms);
  EXPECT_EQ(ticks, after);
}
