#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <grpcpp/grpcpp.h>

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

/**
 * @brief Server-streaming reactor that forwards sensor samples to the single
 * connected client through a bounded FIFO.
 *
 * The driver invokes the callback on its sampling thread. At most one write is
 * in flight; samples arriving meanwhile are appended to a queue of `capacity`
 * samples (at least 1). When it is full the oldest queued sample is dropped
 * and counted, so a slow client skips samples instead of blocking the sensor.
 * With capacity 1 this is "latest wins". Queued samples are converted only
 * when selected for writing, oldest first. The drop total is logged once when
 * the stream ends, never per sample.
 *
 * Thread safety: onSample() runs on the driver's thread, OnWriteDone() and
 * OnCancel() on gRPC threads, possibly concurrently. All reactor state and
 * every StartWrite()/Finish() call are serialised by one mutex, and no write
 * is started once the stream is done (cancelled or a write failed), as the
 * callback API requires. OnDone() relies on the driver's setXCallback({})
 * returning only once no callback is running (see CallbackSlot).
 *
 * @tparam Base the gRPC reactor base class; unit tests substitute a fake that
 * checks the StartWrite()/Finish() protocol.
 */
template <class Sample, class Response,
          class Base = grpc::ServerWriteReactor<Response>>
class SensorStreamReactor : public Base {
public:
  using Callback = std::function<void(Sample)>;
  /// Installs a callback on the driver; an empty callback clears it.
  using Install = std::function<void(Callback)>;
  using Convert = std::function<void(const Sample &, Response &)>;

  SensorStreamReactor(Install install, Convert convert,
                      std::atomic<bool> &in_use, std::string name,
                      size_t capacity)
      : install_(std::move(install)), convert_(std::move(convert)),
        in_use_(in_use), name_(std::move(name)),
        capacity_(std::max<size_t>(1, capacity)) {
    log::info("Start {} stream (queue {}).", name_, capacity_);
    install_([this](Sample sample) { onSample(std::move(sample)); });
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
    if (done_ || queue_.empty()) {
      return;
    }
    convert_(queue_.front(), current_);
    queue_.pop_front();
    startWrite();
  }

  void OnCancel() override {
    std::lock_guard lock(m_);
    done_ = true;
    queue_.clear();
    finish();
  }

  void OnDone() override {
    // Blocks until an in-flight driver callback (if any) has returned, and
    // guarantees no further one starts.
    install_({});
    in_use_.store(false, std::memory_order_release);
    // Lock anyway: dropped_ is guarded by m_.
    size_t dropped;
    {
      std::lock_guard lock(m_);
      dropped = dropped_;
    }
    if (dropped != 0) {
      log::info("{} stream dropped {} samples for a slow client.", name_,
                dropped);
    }
    log::info("Ending {} stream.", name_);
    delete this;
  }

private:
  void onSample(Sample sample) {
    std::lock_guard lock(m_);
    if (done_) {
      return;
    }
    if (writing_) {
      if (queue_.size() >= capacity_) {
        queue_.pop_front();
        ++dropped_;
      }
      queue_.push_back(std::move(sample));
      return;
    }
    convert_(sample, current_);
    startWrite();
  }

  // Both helpers require m_.
  void startWrite() {
    writing_ = true;
    this->StartWrite(&current_);
  }

  void finish() {
    if (!finished_) {
      finished_ = true;
      this->Finish(grpc::Status::OK);
    }
  }

  Install install_;
  Convert convert_;
  std::atomic<bool> &in_use_;
  std::string name_;
  const size_t capacity_;
  std::mutex m_;
  // Everything below is guarded by m_.
  Response current_;         // owned by gRPC while writing_ is true
  std::deque<Sample> queue_; // oldest first, at most capacity_ samples
  size_t dropped_ = 0;
  bool writing_ = false;
  bool done_ = false;
  bool finished_ = false;
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
 * @brief Creates the reactor for a streaming RPC over a single-client sensor.
 *
 * Returns an ErrorReactor with UNAVAILABLE (`unavailable_message`) if there is
 * no driver, with RESOURCE_EXHAUSTED if a stream is already active
 * (`in_use`), and otherwise a SensorStreamReactor that is released by gRPC.
 *
 * @param set_callback pointer to the driver's `setXCallback` member
 * @param convert `void(const Sample &, Response &)`
 * @param name stream name used in the log lines
 * @param requested the client's `max_queued_samples` (0 = default)
 * @param limits default and cap of this stream; see resolveQueueSize()
 */
template <class Sample, class Response, class Driver, class Convert>
grpc::ServerWriteReactor<Response> *
makeStreamReactor(const std::shared_ptr<Driver> &driver,
                  std::atomic<bool> &in_use, std::string unavailable_message,
                  void (Driver::*set_callback)(std::function<void(Sample)>),
                  Convert convert, std::string name, uint32_t requested,
                  QueueLimits limits) {
  if (!driver) {
    return new ErrorReactor<Response>(
        {grpc::StatusCode::UNAVAILABLE, std::move(unavailable_message)});
  }
  if (in_use.exchange(true)) {
    return new ErrorReactor<Response>({grpc::StatusCode::RESOURCE_EXHAUSTED,
                                       "Only one client per stream supported"});
  }
  const size_t capacity = resolveQueueSize(requested, limits);
  if (requested > limits.cap) {
    log::info("{} stream: requested queue {} exceeds the cap, using {}.", name,
              requested, capacity);
  }
  Driver *raw = driver.get();
  return new SensorStreamReactor<Sample, Response>(
      [raw, set_callback](auto callback) {
        (raw->*set_callback)(std::move(callback));
      },
      std::move(convert), in_use, std::move(name), capacity);
}

} // namespace msensor
