#include "shutdown_signal.hh"

#include <atomic>
#include <chrono>
#include <csignal>
#include <future>
#include <gtest/gtest.h>
#include <thread>

using namespace std::chrono_literals;

TEST(ShutdownSignalTest, ReturnsWhenSigtermArrives) {
  // Until waitForShutdownSignal() installs its handler, SIGTERM must not kill
  // the test process.
  const auto old_term = std::signal(SIGTERM, SIG_IGN);

  std::atomic<bool> done{false};
  std::atomic<bool> give_up{false};
  // Raise repeatedly until the wait returns: a signal sent before the handler
  // is installed is simply ignored, so there is no startup race and no fixed
  // sleep. Bounded, so a broken implementation cannot hang the test.
  std::thread raiser([&] {
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (!done && !give_up && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(5ms);
      std::raise(SIGTERM);
    }
  });

  auto waiter = std::async(std::launch::async, [&] {
    msensor::waitForShutdownSignal();
    done = true;
  });
  const bool returned = waiter.wait_for(15s) == std::future_status::ready;
  if (!returned) {
    // Failure path: stop the raiser; the waiter is released by one more
    // signal below so the future's destructor cannot block forever.
    give_up = true;
  }
  raiser.join();
  if (!returned) {
    std::raise(SIGTERM);
  }
  waiter.wait();
  EXPECT_TRUE(returned) << "waitForShutdownSignal() did not return on SIGTERM";
  std::signal(SIGTERM, old_term);
}
