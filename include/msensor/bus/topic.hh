#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "msensor/log/log.hh"

namespace msensor {

template <class T> class Subscription;
template <class T> class Topic;

namespace detail {
template <class T> class TopicCore;
}

/**
 * @brief One consumer's bounded FIFO on a Topic.
 *
 * Created by Topic::subscribe(). Holds `shared_ptr<const T>` samples, oldest
 * first. When the FIFO is full the oldest entry is dropped and counted, so a
 * slow consumer only ever loses its own samples.
 *
 * A consumer either owns a thread and blocks in pop(), or installs a wake-up
 * hook with setWakeHook() and calls tryPop() from it. A hook runs on the
 * topic's dispatcher thread, never on the publisher's thread. It must return
 * promptly (it delays the hooks of the topic's other subscribers), and it must
 * not subscribe to, or destroy a subscription of, the same topic.
 *
 * Destroying a subscription detaches it: when the destructor returns, its hook
 * is not running and will not run again. The last subscription to go clears
 * the driver's callback.
 */
template <class T> class Subscription {
public:
  using Sample = std::shared_ptr<const T>;

  Subscription(const Subscription &) = delete;
  Subscription &operator=(const Subscription &) = delete;
  ~Subscription();

  /// Oldest queued sample, or null if there is none. Never blocks.
  Sample tryPop() {
    Sample out;
    std::lock_guard lock(m_);
    if (!queue_.empty()) {
      out = std::move(queue_.front());
      queue_.pop_front();
    }
    return out;
  }

  /// Oldest queued sample; waits for one. Returns null if `stop` is requested
  /// first.
  Sample pop(std::stop_token stop) {
    std::unique_lock lock(m_);
    if (!cv_.wait(lock, stop, [this] { return !queue_.empty(); })) {
      return nullptr;
    }
    Sample out = std::move(queue_.front());
    queue_.pop_front();
    return out;
  }

  /// Samples dropped so far because the FIFO was full.
  size_t dropped() const {
    std::lock_guard lock(m_);
    return dropped_;
  }

  /// Samples currently queued.
  size_t size() const {
    std::lock_guard lock(m_);
    return queue_.size();
  }

  size_t capacity() const noexcept { return capacity_; }

  /// Installs the wake-up hook. It is called on the dispatcher thread after
  /// samples have arrived (possibly once for several samples) and is called
  /// again for every later arrival. Call it once, right after subscribing; it
  /// also fires for samples that are already queued.
  void setWakeHook(std::function<void()> hook);

private:
  friend class detail::TopicCore<T>;

  Subscription(std::shared_ptr<detail::TopicCore<T>> core, size_t capacity)
      : core_(std::move(core)), capacity_(std::max<size_t>(1, capacity)) {}

  /// Called by the publisher. Returns true if the dispatcher must be woken.
  bool push(const Sample &sample) {
    Sample displaced; // released after the lock, off the hot path
    {
      std::lock_guard lock(m_);
      if (queue_.size() >= capacity_) {
        displaced = std::move(queue_.front());
        queue_.pop_front();
        ++dropped_;
      }
      queue_.push_back(sample);
    }
    cv_.notify_one();
    if (hooked_.load(std::memory_order_acquire)) {
      pending_.store(true, std::memory_order_release);
      return true;
    }
    return false;
  }

  void runHook() {
    std::function<void()> hook;
    {
      std::lock_guard lock(hook_m_);
      hook = hook_;
    }
    if (!hook) {
      return;
    }
    try {
      hook();
    } catch (const std::exception &e) {
      log::error("Subscription wake-up hook threw: {}", e.what());
    } catch (...) {
      log::error("Subscription wake-up hook threw.");
    }
  }

  const std::shared_ptr<detail::TopicCore<T>> core_;
  const size_t capacity_;

  mutable std::mutex m_;
  std::condition_variable_any cv_;
  std::deque<Sample> queue_; // oldest first; guarded by m_
  size_t dropped_ = 0;       // guarded by m_

  std::mutex hook_m_;
  std::function<void()> hook_; // guarded by hook_m_
  std::atomic<bool> hooked_{false};
  std::atomic<bool> pending_{false}; // a hook call is owed
};

namespace detail {

template <class T>
class TopicCore : public std::enable_shared_from_this<TopicCore<T>> {
public:
  using Callback = std::function<void(T)>;
  using Install = std::function<void(Callback)>;

  TopicCore(std::string name, Install install)
      : name_(std::move(name)), install_(std::move(install)) {}

  TopicCore(const TopicCore &) = delete;
  TopicCore &operator=(const TopicCore &) = delete;

  const std::string &name() const noexcept { return name_; }

  std::unique_ptr<Subscription<T>> subscribe(size_t capacity) {
    std::unique_ptr<Subscription<T>> sub(
        new Subscription<T>(this->shared_from_this(), capacity));
    std::lock_guard attach(attach_m_);
    bool first;
    {
      std::lock_guard lock(subs_m_);
      subs_.push_back(sub.get());
      first = subs_.size() == 1;
    }
    if (first) {
      // On failure `sub` detaches itself on the way out.
      dispatcher_ = std::jthread([this](std::stop_token st) { loop(st); });
      if (install_) {
        install_([this](T sample) { publish(std::move(sample)); });
        attached_ = true;
      }
    }
    return sub;
  }

  /// Runs on the driver's thread: wraps the sample once, queues it for every
  /// subscriber and wakes the dispatcher. Nothing else.
  void publish(T sample) {
    bool wake = false;
    {
      std::lock_guard lock(subs_m_);
      if (subs_.empty()) {
        return;
      }
      const std::shared_ptr<const T> shared =
          std::make_shared<const T>(std::move(sample));
      for (Subscription<T> *sub : subs_) {
        wake |= sub->push(shared);
      }
    }
    if (wake) {
      notifyDispatcher();
    }
  }

  size_t subscriberCount() const {
    std::lock_guard lock(subs_m_);
    return subs_.size();
  }

  bool attached() const {
    std::lock_guard lock(attach_m_);
    return attached_;
  }

  void notifyDispatcher() {
    {
      std::lock_guard lock(wake_m_);
      dirty_ = true;
    }
    wake_cv_.notify_one();
  }

  void remove(Subscription<T> *sub) {
    {
      std::lock_guard attach(attach_m_);
      bool empty;
      {
        std::lock_guard lock(subs_m_);
        std::erase(subs_, sub);
        empty = subs_.empty();
      }
      if (empty) {
        // Blocks until a running driver callback has returned. Not under
        // subs_m_, which that callback needs.
        if (attached_) {
          install_({});
          attached_ = false;
        }
        if (dispatcher_.joinable()) {
          dispatcher_.request_stop();
          dispatcher_.join();
          std::lock_guard lock(wake_m_);
          dirty_ = false;
        }
      }
    }
    // A dispatch pass that already picked `sub` holds dispatch_m_ until its
    // hook returns.
    std::lock_guard barrier(dispatch_m_);
  }

private:
  void loop(std::stop_token st) {
    std::unique_lock lock(wake_m_);
    while (wake_cv_.wait(lock, st, [this] { return dirty_; })) {
      dirty_ = false;
      lock.unlock();
      dispatchPass();
      lock.lock();
    }
  }

  void dispatchPass() {
    std::lock_guard pass(dispatch_m_);
    ready_.clear();
    {
      std::lock_guard lock(subs_m_);
      for (Subscription<T> *sub : subs_) {
        if (sub->pending_.exchange(false, std::memory_order_acq_rel)) {
          ready_.push_back(sub);
        }
      }
    }
    for (Subscription<T> *sub : ready_) {
      sub->runHook();
    }
  }

  const std::string name_;
  const Install install_;

  mutable std::mutex attach_m_; // serialises subscribe/remove (driver install)
  bool attached_ = false;       // guarded by attach_m_
  std::jthread dispatcher_;     // guarded by attach_m_

  mutable std::mutex subs_m_; // the list; held by publish for its whole loop
  std::vector<Subscription<T> *> subs_;

  std::mutex dispatch_m_;                // held for a whole dispatch pass
  std::vector<Subscription<T> *> ready_; // guarded by dispatch_m_

  std::mutex wake_m_;
  std::condition_variable_any wake_cv_;
  bool dirty_ = false; // guarded by wake_m_
};

} // namespace detail

template <class T> Subscription<T>::~Subscription() { core_->remove(this); }

template <class T>
void Subscription<T>::setWakeHook(std::function<void()> hook) {
  {
    std::lock_guard lock(hook_m_);
    hook_ = std::move(hook);
  }
  hooked_.store(true, std::memory_order_release);
  bool queued;
  {
    std::lock_guard lock(m_);
    queued = !queue_.empty();
  }
  if (queued) {
    pending_.store(true, std::memory_order_release);
    core_->notifyDispatcher();
  }
}

/**
 * @brief Fan-out point between one producer and any number of subscribers.
 *
 * The producer is a driver whose single `setXCallback` consumer is the topic:
 * `install` is called with a callback when the first subscriber arrives and
 * with an empty one when the last leaves, so a driver with no subscriber has
 * no callback (some drivers then skip their work). Without an `install`
 * function the owner calls publish() itself.
 *
 * publish() runs on the producer's thread. It wraps the sample in a
 * `shared_ptr<const T>` once, pushes it into each subscription's bounded FIFO
 * and wakes the dispatcher thread. It does not convert, do I/O, or wait on a
 * subscriber. The topic runs one dispatcher thread while it has subscribers;
 * that thread calls the subscriptions' wake-up hooks.
 *
 * Subscriptions keep the topic's state alive, so they may outlive the Topic
 * object; the driver is then still attached until the last one is destroyed.
 */
template <class T> class Topic {
public:
  using Callback = std::function<void(T)>;
  using Install = std::function<void(Callback)>;

  explicit Topic(std::string name = {}, Install install = {})
      : core_(std::make_shared<detail::TopicCore<T>>(std::move(name),
                                                     std::move(install))) {}

  Topic(const Topic &) = delete;
  Topic &operator=(const Topic &) = delete;

  /// New subscription with a FIFO of `capacity` samples (at least 1).
  std::unique_ptr<Subscription<T>> subscribe(size_t capacity) {
    return core_->subscribe(capacity);
  }

  /// Hands a sample to every subscriber; see the class comment.
  void publish(T sample) { core_->publish(std::move(sample)); }

  size_t subscriberCount() const { return core_->subscriberCount(); }

  /// True while the topic is installed as the driver's callback.
  bool attached() const { return core_->attached(); }

  const std::string &name() const noexcept { return core_->name(); }

private:
  std::shared_ptr<detail::TopicCore<T>> core_;
};

} // namespace msensor
