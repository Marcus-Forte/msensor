#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <memory>
#include <string>
#include <utility>

#include <grpcpp/grpcpp.h>

#include "msensor/log/log.hh"

namespace msensor {

/**
 * @brief Server-streaming reactor that forwards the latest sensor sample to
 * the single connected client.
 *
 * The driver invokes the callback on its sampling thread. At most one write is
 * in flight; samples arriving meanwhile overwrite a single pending slot, so a
 * slow client skips samples instead of blocking the sensor. Pending samples
 * are converted only when selected for writing.
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
                      std::atomic<bool> &in_use, std::string name)
      : install_(std::move(install)), convert_(std::move(convert)),
        in_use_(in_use), name_(std::move(name)) {
    log::info("Start {} stream.", name_);
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
    if (done_ || !next_) {
      return;
    }
    convert_(*next_, current_);
    next_.reset();
    startWrite();
  }

  void OnCancel() override {
    std::lock_guard lock(m_);
    done_ = true;
    next_.reset();
    finish();
  }

  void OnDone() override {
    // Blocks until an in-flight driver callback (if any) has returned, and
    // guarantees no further one starts.
    install_({});
    in_use_.store(false, std::memory_order_release);
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
      next_ = std::move(sample);
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
  std::mutex m_;
  // Everything below is guarded by m_.
  Response current_; // owned by gRPC while writing_ is true
  std::optional<Sample> next_;
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
 */
template <class Sample, class Response, class Driver, class Convert>
grpc::ServerWriteReactor<Response> *makeStreamReactor(
    const std::shared_ptr<Driver> &driver, std::atomic<bool> &in_use,
    std::string unavailable_message,
    void (Driver::*set_callback)(std::function<void(Sample)>), Convert convert,
    std::string name) {
  if (!driver) {
    return new ErrorReactor<Response>(
        {grpc::StatusCode::UNAVAILABLE, std::move(unavailable_message)});
  }
  if (in_use.exchange(true)) {
    return new ErrorReactor<Response>({grpc::StatusCode::RESOURCE_EXHAUSTED,
                                       "Only one client per stream supported"});
  }
  Driver *raw = driver.get();
  return new SensorStreamReactor<Sample, Response>(
      [raw, set_callback](auto callback) {
        (raw->*set_callback)(std::move(callback));
      },
      std::move(convert), in_use, std::move(name));
}

} // namespace msensor
