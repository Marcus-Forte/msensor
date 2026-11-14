#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stop_token>
#include <thread>
#include <type_traits>
#include <utility>

namespace msensor {

/**
 * @brief Owns a sampling thread for a driver.
 *
 * Two modes:
 *  - `startPeriodic(period, tick)` calls `tick()` on a fixed cadence: tick n is
 *    due at `start + n * period` (the first one immediately). If a tick
 *    overruns its slot, the missed slots are skipped, not fired in a burst.
 *  - `startFreeRunning(step)` calls `step()` back to back, for work that is
 *    itself a blocking read. `step()` returns how long to back off before the
 *    next call (zero for none).
 *
 * All waits are stop-aware, so `stop()` returns promptly whatever the period
 * or back-off.
 *
 * `start...()` is a no-op while running; `stop()` is a no-op when not
 * running; starting again after `stop()` works. The destructor stops and
 * joins. start/stop/running must be called from one controlling thread.
 *
 * Declare it as the LAST data member of a driver: it is joined before the
 * members the loop uses are destroyed.
 */
class SamplingThread {
public:
  using Clock = std::chrono::steady_clock;

  SamplingThread() = default;
  SamplingThread(const SamplingThread &) = delete;
  SamplingThread &operator=(const SamplingThread &) = delete;
  ~SamplingThread() { stop(); }

  /// Call `tick()` every `period` (must be positive). No-op if running.
  template <typename Tick>
  void startPeriodic(std::chrono::nanoseconds period, Tick tick) {
    if (thread_.joinable()) {
      return;
    }
    thread_ = std::jthread(
        [period, tick = std::move(tick)](std::stop_token st) mutable {
          runPeriodic(st, period, tick);
        });
  }

  /// Call `step()` repeatedly; it returns the back-off before the next call.
  /// No-op if running.
  template <typename Step> void startFreeRunning(Step step) {
    if (thread_.joinable()) {
      return;
    }
    thread_ = std::jthread([step = std::move(step)](std::stop_token st) mutable {
      runFreeRunning(st, step);
    });
  }

  /// Request stop and join. No-op if not running.
  void stop() {
    if (!thread_.joinable()) {
      return;
    }
    thread_.request_stop();
    thread_.join();
  }

  bool running() const { return thread_.joinable(); }

private:
  template <typename Tick>
  static void runPeriodic(std::stop_token st, std::chrono::nanoseconds period,
                          Tick &tick) {
    std::mutex m;
    std::condition_variable_any cv;
    auto next = Clock::now();
    while (!st.stop_requested()) {
      tick();
      next += period;
      const auto now = Clock::now();
      if (next <= now) {
        // Overrun: skip the missed slots, stay on the original grid.
        next += period * ((now - next) / period + 1);
      }
      std::unique_lock lock(m);
      cv.wait_until(lock, st, next, [] { return false; });
    }
  }

  template <typename Step>
  static void runFreeRunning(std::stop_token st, Step &step) {
    std::mutex m;
    std::condition_variable_any cv;
    while (!st.stop_requested()) {
      const std::chrono::nanoseconds backoff = step();
      if (backoff > std::chrono::nanoseconds::zero()) {
        std::unique_lock lock(m);
        cv.wait_for(lock, st, backoff, [] { return false; });
      }
    }
  }

  std::jthread thread_;
};

} // namespace msensor
