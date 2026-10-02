#include "lidar_service.hh"
#include "msensor/conversions/conversions.hh"
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>

LidarServiceImpl::LidarServiceImpl(std::shared_ptr<msensor::ILidar> lidar)
    : lidar_(lidar) {}

namespace {

/**
 * Base for scan-streaming reactors. Subscribes to the scan hub and writes the
 * freshest scan whenever the previous write has completed.
 *
 * - The hub's notify callback runs on the sensor's producer thread, so it only
 *   schedules work on the TaskRunner.
 * - At most one write is in flight (`writing_`). Scans that arrive meanwhile
 *   overwrite each other in the subscription, so a slow client skips scans.
 *
 * Call begin() once after construction.
 */
template <class Base> class ScanStreamReactor : public Base {
public:
  ScanStreamReactor(msensor::SensorHub<msensor::Scan3DI> &hub,
                    TaskRunner &runner)
      : hub_(hub), runner_(runner) {}

  void OnWriteDone(bool ok) override {
    if (!ok || done_.load()) {
      done_ = true; // writing_ stays set, so nothing else is written
      finish();
      return;
    }
    writing_ = false;
    pump();
  }

  void OnCancel() override {
    done_ = true;
    // If no write is in flight nobody else will end the RPC. Otherwise the
    // write's completion does.
    if (!writing_.exchange(true)) {
      finish();
    }
  }

  void OnDone() override {
    // After this returns the hub no longer calls us and no task is running.
    sub_->setNotify({});
    {
      std::unique_lock lock(tasks_m_);
      tasks_cv_.wait(lock, [this] { return tasks_ == 0; });
    }
    delete this;
  }

protected:
  void begin() {
    sub_ = hub_.subscribe(msensor::SubscribePolicy::latestOnly());
    sub_->setNotify([this] { onScan(); });
    pump();
  }

  /// Fill `out` from `scan`. Runs on the TaskRunner or a gRPC thread.
  virtual void convert(const std::shared_ptr<const msensor::Scan3DI> &scan,
                       sensors::PointCloud3 &out) = 0;

private:
  /// Ends the RPC, once.
  void finish() {
    if (!finished_.exchange(true)) {
      this->Finish(grpc::Status::OK);
    }
  }

  void onScan() {
    if (writing_.load() || queued_.exchange(true))
      return;
    {
      std::lock_guard lock(tasks_m_);
      ++tasks_;
    }
    runner_.post([this] {
      queued_ = false;
      pump();
      {
        std::lock_guard lock(tasks_m_);
        --tasks_;
      }
      tasks_cv_.notify_all();
    });
  }

  void pump() {
    while (true) {
      if (writing_.exchange(true))
        return; // a write is in flight, or another pump owns the flag
      if (done_.load()) {
        finish();
        return;
      }
      if (auto scan = sub_->tryPop()) {
        convert(scan, response_);
        this->StartWrite(&response_);
        return;
      }
      writing_ = false;
      if (sub_->empty())
        return;
      // A scan arrived while we held the flag (its notify saw writing_).
    }
  }

  msensor::SensorHub<msensor::Scan3DI> &hub_;
  TaskRunner &runner_;
  sensors::PointCloud3 response_;
  std::atomic<bool> writing_{false};
  std::atomic<bool> done_{false};
  std::atomic<bool> queued_{false};
  std::atomic<bool> finished_{false};
  std::mutex tasks_m_;
  std::condition_variable tasks_cv_;
  int tasks_ = 0;
  std::unique_ptr<msensor::SensorHub<msensor::Scan3DI>::Subscription> sub_;
};

// ---------------------------------------------------------------------------
// getLidarScan — server-streaming via WriteReactor
// ---------------------------------------------------------------------------

using LidarScanStreamBase =
    ScanStreamReactor<grpc::ServerWriteReactor<sensors::PointCloud3>>;

class LidarScanReactor : public LidarScanStreamBase {
public:
  using LidarScanStreamBase::LidarScanStreamBase;

  void start() {
    std::cout << "Start Lidar scan stream." << std::endl;
    begin();
  }

  void OnCancel() override {
    std::cout << "Ending Lidar scan stream." << std::endl;
    LidarScanStreamBase::OnCancel();
  }

protected:
  void convert(const std::shared_ptr<const msensor::Scan3DI> &scan,
               sensors::PointCloud3 &out) override {
    out = toProtobuf(scan);
  }
};

/// Reactor that immediately ends the RPC with UNAVAILABLE.
template <class Base> class UnavailableReactor : public Base {
public:
  UnavailableReactor() {
    this->Finish(
        grpc::Status(grpc::StatusCode::UNAVAILABLE, "Lidar not available"));
  }
  void OnDone() override { delete this; }
};

} // namespace

grpc::ServerWriteReactor<sensors::PointCloud3> *LidarServiceImpl::getLidarScan(
    grpc::CallbackServerContext * /*context*/,
    const sensors::LidarStreamRequest * /*request*/) {
  if (!lidar_) {
    return new UnavailableReactor<
        grpc::ServerWriteReactor<sensors::PointCloud3>>();
  }
  auto *reactor = new LidarScanReactor(lidar_->scans(), runner_);
  reactor->start();
  return reactor;
}
