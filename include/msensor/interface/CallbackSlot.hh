#pragma once

#include <functional>
#include <mutex>
#include <utility>

namespace msensor {

/**
 * @brief Thread-safe holder for the single consumer callback of a sensor.
 *
 * A sensor stream has at most one consumer (the gRPC service serving its one
 * client), so no queue or fan-out is needed: emit() invokes the registered
 * callback directly on the producer's thread.
 *
 * emit() runs the callback while holding the lock, so setCallback() returning
 * guarantees that no callback is running and none will start. A gRPC reactor
 * relies on this to clear the callback and delete itself safely.
 */
template <class T> class CallbackSlot {
public:
  using Callback = std::function<void(T)>;

  /// Install the consumer. Passing an empty callback clears it.
  void setCallback(Callback callback) {
    std::lock_guard lock(mutex_);
    callback_ = std::move(callback);
  }

  /// Deliver a sample to the consumer, if one is registered.
  void emit(T sample) {
    std::lock_guard lock(mutex_);
    if (callback_) {
      callback_(std::move(sample));
    }
  }

  bool hasCallback() const {
    std::lock_guard lock(mutex_);
    return static_cast<bool>(callback_);
  }

private:
  mutable std::mutex mutex_;
  Callback callback_;
};

} // namespace msensor
