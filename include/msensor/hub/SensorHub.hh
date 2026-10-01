#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <stop_token>
#include <utility>
#include <vector>

namespace msensor {

/**
 * @brief Per-subscriber buffering policy.
 *
 * A subscriber keeps at most `depth` messages. When full, the oldest message
 * is dropped (and counted) so a slow subscriber never blocks the producer or
 * other subscribers.
 */
struct SubscribePolicy {
  std::size_t depth = 1;

  /// Keep only the freshest message (lidar, camera).
  static constexpr SubscribePolicy latestOnly() { return {1}; }
  /// Keep a short burst of messages (IMU, ADC, recorder).
  static constexpr SubscribePolicy bounded(std::size_t n) {
    return {n == 0 ? 1 : n};
  }
};

/**
 * @brief Single-producer, multi-subscriber fan-out for immutable sensor data.
 *
 * The producer calls publish() with a `shared_ptr<const T>`; every subscriber
 * receives the same pointer, so fan-out never copies the payload. Each
 * subscriber has its own bounded buffer, so consumers cannot steal data from
 * each other and a slow consumer only loses its own messages.
 *
 * Consumers either poll (tryPop), block (waitPop), or register a notify
 * callback (setNotify) for event-driven use, e.g. from a gRPC reactor.
 *
 * Thread-safety: publish(), subscribe() and all Subscription methods may be
 * called from any thread.
 */
template <class T> class SensorHub {
public:
  using Ptr = std::shared_ptr<const T>;

private:
  struct Slot {
    explicit Slot(SubscribePolicy p) : policy(p) {}

    void push(Ptr msg) {
      {
        std::lock_guard lock(m);
        if (closed)
          return;
        if (queue.size() >= policy.depth) {
          queue.pop_front();
          dropped.fetch_add(1, std::memory_order_relaxed);
        }
        queue.push_back(std::move(msg));
      }
      cv.notify_one();

      // Serialised against close(), so no callback runs after close() returns.
      std::lock_guard nlock(notify_m);
      if (notify)
        notify();
    }

    Ptr tryPop() {
      std::lock_guard lock(m);
      if (queue.empty())
        return nullptr;
      Ptr out = std::move(queue.front());
      queue.pop_front();
      return out;
    }

    void close() {
      {
        std::lock_guard lock(m);
        closed = true;
        queue.clear();
      }
      cv.notify_all();
      std::lock_guard nlock(notify_m);
      notify = nullptr;
    }

    const SubscribePolicy policy;
    std::mutex m;
    std::condition_variable_any cv;
    std::deque<Ptr> queue;
    bool closed = false;
    std::atomic<uint64_t> dropped{0};

    std::mutex notify_m;
    std::function<void()> notify;
  };

public:
  /**
   * @brief Handle to one subscriber's buffer. Unsubscribes on destruction.
   *
   * A Subscription is meant to be used by a single consumer.
   */
  class Subscription {
  public:
    ~Subscription() { slot_->close(); }
    Subscription(const Subscription &) = delete;
    Subscription &operator=(const Subscription &) = delete;

    /// Non-blocking. Returns the oldest buffered message or nullptr.
    Ptr tryPop() { return slot_->tryPop(); }

    /**
     * @brief Block until a message is available, the timeout expires, or stop
     * is requested.
     * @return The message, or nullptr on timeout / stop.
     */
    Ptr waitPop(std::stop_token stop, std::chrono::milliseconds timeout) {
      std::unique_lock lock(slot_->m);
      slot_->cv.wait_for(lock, stop, timeout,
                         [&] { return !slot_->queue.empty(); });
      if (slot_->queue.empty())
        return nullptr;
      Ptr out = std::move(slot_->queue.front());
      slot_->queue.pop_front();
      return out;
    }

    /**
     * @brief Register a "data available" callback.
     *
     * Runs on the producer thread after each publish, so it must be cheap and
     * non-blocking (typically: kick a reactor, which then calls tryPop()).
     * It may call tryPop() but must not destroy this Subscription or call
     * setNotify() itself. Passing an empty function clears it; once
     * setNotify()/the destructor returns, the previous callback is not
     * running and will not run again.
     *
     * Messages published before the callback was set are not signalled; call
     * tryPop() once after registering to pick them up.
     */
    void setNotify(std::function<void()> fn) {
      std::lock_guard lock(slot_->notify_m);
      slot_->notify = std::move(fn);
    }

    /// True if no message is currently buffered.
    bool empty() const {
      std::lock_guard lock(slot_->m);
      return slot_->queue.empty();
    }

    /// Number of messages dropped because this subscriber's buffer was full.
    uint64_t dropped() const {
      return slot_->dropped.load(std::memory_order_relaxed);
    }

  private:
    friend class SensorHub;
    explicit Subscription(std::shared_ptr<Slot> slot)
        : slot_(std::move(slot)) {}
    std::shared_ptr<Slot> slot_;
  };

  /// Create a new subscriber. Only messages published afterwards are seen.
  std::unique_ptr<Subscription>
  subscribe(SubscribePolicy policy = SubscribePolicy::latestOnly()) {
    auto slot = std::make_shared<Slot>(policy);
    {
      std::lock_guard lock(m_);
      prune();
      slots_.push_back(slot);
    }
    return std::unique_ptr<Subscription>(new Subscription(std::move(slot)));
  }

  /// Deliver a message to every current subscriber. Never blocks on consumers.
  void publish(Ptr msg) {
    std::vector<std::shared_ptr<Slot>> targets;
    {
      std::lock_guard lock(m_);
      prune();
      targets.reserve(slots_.size());
      for (auto &w : slots_)
        if (auto s = w.lock())
          targets.push_back(std::move(s));
    }
    for (auto &s : targets)
      s->push(msg);
    published_.fetch_add(1, std::memory_order_relaxed);
  }

  void publish(T value) {
    publish(std::make_shared<const T>(std::move(value)));
  }

  std::size_t subscriberCount() const {
    std::lock_guard lock(m_);
    return static_cast<std::size_t>(
        std::count_if(slots_.begin(), slots_.end(),
                      [](const auto &w) { return !w.expired(); }));
  }

  uint64_t publishedCount() const {
    return published_.load(std::memory_order_relaxed);
  }

private:
  void prune() {
    std::erase_if(slots_, [](const auto &w) { return w.expired(); });
  }

  mutable std::mutex m_;
  std::vector<std::weak_ptr<Slot>> slots_;
  std::atomic<uint64_t> published_{0};
};

} // namespace msensor
