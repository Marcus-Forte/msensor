#include "shutdown_signal.hh"

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

namespace msensor {

namespace {
// A lock-free atomic is async-signal-safe and, unlike volatile sig_atomic_t,
// also well defined when the handler runs on a different thread than the
// waiter (a process-directed signal may be delivered to any thread).
std::atomic<bool> g_shutdown_requested{false};
static_assert(std::atomic<bool>::is_always_lock_free);

extern "C" void onShutdownSignal(int /*signal*/) {
  g_shutdown_requested.store(true, std::memory_order_relaxed);
}
} // namespace

void waitForShutdownSignal() {
  g_shutdown_requested.store(false);
  const auto old_int = std::signal(SIGINT, onShutdownSignal);
  const auto old_term = std::signal(SIGTERM, onShutdownSignal);

  while (!g_shutdown_requested.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::signal(SIGINT, old_int);
  std::signal(SIGTERM, old_term);
}

} // namespace msensor
