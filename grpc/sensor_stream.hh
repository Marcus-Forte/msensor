#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <grpcpp/grpcpp.h>

#include "msensor/bus/topic.hh"
#include "msensor/log/log.hh"

namespace msensor {

/// Default and maximum depth of a stream's queue of samples waiting for a slow
/// client. The client asks through `max_queued_samples` in its request.
struct QueueLimits {
  uint32_t default_size; ///< Used when the client asks for 0.
  uint32_t cap;          ///< Largest depth the server grants.
};

/// IMU: every sample matters to a SLAM; 200 is about 1 s at 200 Hz.
inline constexpr QueueLimits kImuQueueLimits{200, 4000};
/// LiDAR: a Mid360 scan of 9600 points is roughly 150 KiB in memory.
inline constexpr QueueLimits kLidarQueueLimits{1, 64};
/// Camera: raw frames are several MB.
inline constexpr QueueLimits kCameraQueueLimits{1, 8};

/// Depth granted for a requested `max_queued_samples`: 0 gives the default,
/// anything above the cap gives the cap. Never returns less than 1.
constexpr size_t resolveQueueSize(uint32_t requested, QueueLimits limits) {
  const uint32_t size = requested == 0 ? limits.default_size : requested;
  return std::max<uint32_t>(1, std::min(size, limits.cap));
}

/// Clients one stream serves at once unless `ServerOptions` says otherwise.
inline constexpr size_t kDefaultMaxClientsPerStream = 8;

/// Counts the clients of one stream against a maximum.
class ClientLimit {
public:
  explicit ClientLimit(size_t max_clients) : max_(max_clients) {}

  /// Takes a slot; false if all of them are taken.
  bool tryAcquire() {
    size_t current = active_.load(std::memory_order_relaxed);
    while (current < max_) {
      if (active_.compare_exchange_weak(current, current + 1,
                                        std::memory_order_acq_rel)) {
        return true;
      }
    }
    return false;
  }

  void release() { active_.fetch_sub(1, std::memory_order_acq_rel); }

  size_t active() const { return active_.load(std::memory_order_acquire); }
  size_t max() const noexcept { return max_; }

private:
  const size_t max_;
  std::atomic<size_t> active_{0};
};

/**
 * @brief Server-streaming reactor that forwards the samples of a bus topic to
 * one client through the bounded FIFO of its subscription.
 *
 * The subscription's FIFO holds `capacity` samples (at least 1); when it is
 * full the oldest is dropped and counted, so a slow client skips samples
 * instead of blocking the sensor. With capacity 1 this is "latest wins". At
 * most one write is in flight. Samples are converted to protobuf only when
 * they are selected for writing, oldest first, so a dropped sample is never
 * converted.
 *
 * Threads: the driver's thread only publishes to the topic (see Topic). The
 * wake-up hook runs on the topic's dispatcher thread; OnWriteDone() and
 * OnCancel() run on gRPC threads. Conversion and every StartWrite()/Finish()
 * call happen on those, serialised by one mutex, and no write is started once
 * the stream is done (cancelled or a write failed), as the callback API
 * requires. OnDone() destroys the subscription, which guarantees the hook is
 * not running, and then deletes the reactor.
 *
 * Drops are logged while they happen, at most once per second, and their total
 * when the stream ends.
 *
 * @tparam Base the gRPC reactor base class; unit tests substitute a fake that
 * checks the StartWrite()/Finish() protocol.
 */
template <class Sample, class Response,
          class Base = grpc::ServerWriteReactor<Response>>
class SensorStreamReactor : public Base {
public:
  using Convert = std::function<void(const Sample &, Response &)>;

  /// `limit` must already hold a slot for this stream; OnDone() releases it.
  SensorStreamReactor(std::unique_ptr<Subscription<Sample>> subscription,
                      Convert convert, ClientLimit &limit, std::string name)
      : sub_(std::move(subscription)), convert_(std::move(convert)),
        limit_(limit), name_(std::move(name)) {
    log::info("Start {} stream (queue {}).", name_, sub_->capacity());
    sub_->setWakeHook([this] { onWake(); });
  }

  void OnWriteDone(bool ok) override {
    std::lock_guard lock(m_);
    writing_ = false;
    if (!ok) {
      done_ = true;
      finish();
      return;
    }
    // A write may complete successfully after OnCancel() already finished
    // the stream; starting another write then would violate the API.
    if (done_) {
      return;
    }
    reportDrops();
    startNext();
  }

  void OnCancel() override {
    std::lock_guard lock(m_);
    done_ = true;
    while (sub_->tryPop()) {
    }
    finish();
  }

  void OnDone() override {
    const size_t dropped = sub_->dropped();
    // Detaches: when this returns the hook is not running and never will be,
    // and the driver's callback is cleared if this was the last subscriber.
    sub_.reset();
    limit_.release();
    if (dropped != 0) {
      log::info("{} stream dropped {} samples for a slow client.", name_,
                dropped);
    }
    log::info("Ending {} stream.", name_);
    delete this;
  }

private:
  // Dispatcher thread.
  void onWake() {
    std::lock_guard lock(m_);
    if (done_) {
      return;
    }
    reportDrops();
    if (!writing_) {
      startNext();
    }
  }

  // Requires m_ and !writing_ and !done_. Writes the oldest queued sample.
  void startNext() {
    const auto sample = sub_->tryPop();
    if (!sample) {
      return;
    }
    convert_(*sample, current_);
    writing_ = true;
    this->StartWrite(&current_);
  }

  // Requires m_.
  void finish() {
    if (!finished_) {
      finished_ = true;
      this->Finish(grpc::Status::OK);
    }
  }

  // Requires m_. Logs the drops since the last report, at most once a second.
  void reportDrops() {
    const size_t total = sub_->dropped();
    if (total == reported_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (logged_ && now - last_report_ < std::chrono::seconds(1)) {
      return;
    }
    log::warn("{} stream: slow client, dropped {} more samples ({} so far).",
              name_, total - reported_, total);
    reported_ = total;
    last_report_ = now;
    logged_ = true;
  }

  std::unique_ptr<Subscription<Sample>> sub_;
  Convert convert_;
  ClientLimit &limit_;
  std::string name_;
  std::mutex m_;
  // Everything below is guarded by m_.
  Response current_; // owned by gRPC while writing_ is true
  bool writing_ = false;
  bool done_ = false;
  bool finished_ = false;
  size_t reported_ = 0; // drops already logged
  bool logged_ = false;
  std::chrono::steady_clock::time_point last_report_;
};

/// Reactor that immediately ends the RPC with the given status.
template <class Response>
class ErrorReactor : public grpc::ServerWriteReactor<Response> {
public:
  explicit ErrorReactor(grpc::Status status) {
    this->Finish(std::move(status));
  }
  void OnDone() override { delete this; }
};

/**
 * @brief Creates the reactor for a streaming RPC over a bus topic.
 *
 * Returns an ErrorReactor with UNAVAILABLE (`unavailable_message`) if there is
 * no topic (the sensor is not available), with RESOURCE_EXHAUSTED if the
 * stream already has `limit.max()` clients, and otherwise a
 * SensorStreamReactor that is released by gRPC.
 *
 * @param topic the sensor's topic, or null
 * @param limit the stream's client counter
 * @param convert `void(const Sample &, Response &)`
 * @param name stream name used in the log lines
 * @param requested the client's `max_queued_samples` (0 = default)
 * @param limits default and cap of this stream; see resolveQueueSize()
 */
template <class Sample, class Response, class Convert>
grpc::ServerWriteReactor<Response> *
makeStreamReactor(Topic<Sample> *topic, ClientLimit &limit,
                  std::string unavailable_message, Convert convert,
                  std::string name, uint32_t requested, QueueLimits limits) {
  if (!topic) {
    return new ErrorReactor<Response>(
        {grpc::StatusCode::UNAVAILABLE, std::move(unavailable_message)});
  }
  if (!limit.tryAcquire()) {
    return new ErrorReactor<Response>(
        {grpc::StatusCode::RESOURCE_EXHAUSTED,
         "Too many clients on this stream (maximum " +
             std::to_string(limit.max()) + ")"});
  }
  const size_t capacity = resolveQueueSize(requested, limits);
  if (requested > limits.cap) {
    log::info("{} stream: requested queue {} exceeds the cap, using {}.", name,
              requested, capacity);
  }
  std::unique_ptr<Subscription<Sample>> subscription;
  try {
    subscription = topic->subscribe(capacity);
  } catch (...) {
    limit.release();
    throw;
  }
  return new SensorStreamReactor<Sample, Response>(
      std::move(subscription), std::move(convert), limit, std::move(name));
}

} // namespace msensor
