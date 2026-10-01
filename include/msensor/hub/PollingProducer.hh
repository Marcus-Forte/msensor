#pragma once

#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stop_token>
#include <thread>

#include "msensor/hub/SensorHub.hh"

namespace msensor {

/**
 * @brief Runs a thread that repeatedly calls a source function and publishes
 * its results to a SensorHub.
 *
 * Meant for drivers that have no push-style callback (blocking reads, I2C
 * polling, simulators).
 *
 * - With a `period`, the source is called at a fixed rate (the source should
 *   not block for long).
 * - Without a period, the source is called back-to-back, so it is expected to
 *   block until data is ready (with a timeout).
 *
 * A source returning nullptr publishes nothing. A source that throws is logged
 * and retried after a short delay.
 *
 * start() and stop() must not be called concurrently with each other.
 * Declare this as the last member of the owning driver so the thread is
 * stopped before the rest of the driver is destroyed.
 */
template <class T> class PollingProducer {
public:
  using Source = std::function<std::shared_ptr<const T>()>;

  PollingProducer(SensorHub<T> &hub, Source source,
                  std::chrono::nanoseconds period = std::chrono::nanoseconds{0})
      : hub_(hub), source_(std::move(source)), period_(period) {}

  ~PollingProducer() { stop(); }

  PollingProducer(const PollingProducer &) = delete;
  PollingProducer &operator=(const PollingProducer &) = delete;

  /// Start the thread. No-op if already running.
  void start() {
    if (thread_.joinable())
      return;
    thread_ = std::jthread([this](std::stop_token st) { run(st); });
  }

  /// Stop and join the thread. No-op if not running.
  void stop() {
    if (!thread_.joinable())
      return;
    thread_.request_stop();
    thread_.join();
  }

  bool running() const { return thread_.joinable(); }

private:
  using Clock = std::chrono::steady_clock;

  static constexpr auto kIdleDelay = std::chrono::milliseconds(1);
  static constexpr auto kErrorDelay = std::chrono::milliseconds(100);

  void run(std::stop_token st) {
    auto next = Clock::now();
    while (!st.stop_requested()) {
      bool got_data = false;
      try {
        if (auto msg = source_()) {
          hub_.publish(std::move(msg));
          got_data = true;
        }
      } catch (const std::exception &e) {
        std::cerr << "PollingProducer: source failed: " << e.what()
                  << std::endl;
        sleepUntil(st, Clock::now() + kErrorDelay);
        next = Clock::now();
        continue;
      }

      if (period_.count() > 0) {
        next += period_;
        const auto now = Clock::now();
        if (next < now)
          next = now; // fell behind, do not try to catch up
        sleepUntil(st, next);
      } else if (!got_data) {
        sleepUntil(st, Clock::now() + kIdleDelay);
      }
    }
  }

  static void sleepUntil(std::stop_token st, Clock::time_point tp) {
    std::mutex m;
    std::condition_variable_any cv;
    std::unique_lock lock(m);
    cv.wait_until(lock, st, tp, [] { return false; });
  }

  SensorHub<T> &hub_;
  Source source_;
  const std::chrono::nanoseconds period_;
  std::jthread thread_;
};

} // namespace msensor
