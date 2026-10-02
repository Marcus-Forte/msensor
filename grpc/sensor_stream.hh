#pragma once

#include <atomic>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <grpcpp/grpcpp.h>

namespace msensor {

/**
 * @brief Server-streaming reactor that forwards the latest sensor sample to
 * the single connected client.
 *
 * The driver invokes the callback on its sampling thread. At most one write is
 * in flight; samples arriving meanwhile overwrite a single pending slot, so a
 * slow client skips samples instead of blocking the sensor. Pending samples
 * are converted only when selected for writing. All reactor state and gRPC
 * operations are serialised by one mutex, as required by the callback API.
 */
template <class Sample, class Response>
class SensorStreamReactor : public grpc::ServerWriteReactor<Response> {
public:
  using Callback = std::function<void(const Sample &)>;
  /// Installs a callback on the driver; an empty callback clears it.
  using Install = std::function<void(Callback)>;
  using Convert = std::function<void(const Sample &, Response &)>;

  SensorStreamReactor(Install install, Convert convert,
                      std::atomic<bool> &in_use, std::string name)
      : install_(std::move(install)), convert_(std::move(convert)),
        in_use_(in_use), name_(std::move(name)) {
    std::cout << "Start " << name_ << " stream." << std::endl;
    install_([this](const Sample &sample) { onSample(sample); });
  }

  void OnWriteDone(bool ok) override {
    std::lock_guard lock(m_);
    if (!ok) {
      done_ = true;
      finish();
      return;
    }
    writing_ = false;
    if (next_) {
      convert_(*next_, current_);
      next_.reset();
      writing_ = true;
      this->StartWrite(&current_);
    }
  }

  void OnCancel() override {
    std::lock_guard lock(m_);
    done_ = true;
    finish();
  }

  void OnDone() override {
    // Blocks until an in-flight driver callback (if any) has returned, and
    // guarantees no further one starts.
    install_({});
    in_use_.store(false, std::memory_order_release);
    std::cout << "Ending " << name_ << " stream." << std::endl;
    delete this;
  }

private:
  void onSample(const Sample &sample) {
    std::lock_guard lock(m_);
    if (done_) {
      return;
    }
    if (writing_) {
      next_ = sample;
      return;
    }
    convert_(sample, current_);
    writing_ = true;
    this->StartWrite(&current_);
  }

  void finish() {
    if (!finished_.exchange(true)) {
      this->Finish(grpc::Status::OK);
    }
  }

  Install install_;
  Convert convert_;
  std::atomic<bool> &in_use_;
  std::string name_;
  std::mutex m_;
  Response current_;
  std::optional<Sample> next_;
  bool writing_ = false;
  bool done_ = false;
  std::atomic<bool> finished_{false};
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

} // namespace msensor
