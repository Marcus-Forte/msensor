#include "lidar_service.hh"
#include "msensor/conversions/conversions.hh"
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <pcl/filters/voxel_grid.h>

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

class LidarScanReactor
    : public ScanStreamReactor<grpc::ServerWriteReactor<sensors::PointCloud3>> {
public:
  using ScanStreamReactor::ScanStreamReactor;

  void start() {
    std::cout << "Start Lidar scan stream." << std::endl;
    begin();
  }

  void OnCancel() override {
    std::cout << "Ending Lidar scan stream." << std::endl;
    ScanStreamReactor::OnCancel();
  }

protected:
  void convert(const std::shared_ptr<const msensor::Scan3DI> &scan,
               sensors::PointCloud3 &out) override {
    out = toProtobuf(scan);
  }
};

// ---------------------------------------------------------------------------
// getSubSampledLidarScan — bidi streaming via BidiReactor
//
// Reads and writes are fully independent:
//   - OnReadDone:  updates the voxel size when the client sends a new value
//   - writes:      the freshest scan is filtered and written back
// ---------------------------------------------------------------------------

class SubSampledLidarReactor
    : public ScanStreamReactor<grpc::ServerBidiReactor<
          sensors::SubSampledLidarStreamRequest, sensors::PointCloud3>> {
public:
  using ScanStreamReactor::ScanStreamReactor;

  void start() {
    std::cout << "Start subsampled Lidar scan stream." << std::endl;
    StartRead(&request_); // start listening for client messages
    begin();
  }

  void OnReadDone(bool ok) override {
    if (!ok)
      return; // client closed its half
    const float vs = request_.voxel_size();
    if (std::isfinite(vs) && vs > 0.0f) {
      voxel_size_.store(vs);
    } else {
      std::cerr << "Ignoring invalid voxel size: " << vs << std::endl;
    }
    StartRead(&request_); // keep listening
  }

  void OnCancel() override {
    std::cout << "Ending subsampled Lidar scan stream." << std::endl;
    ScanStreamReactor::OnCancel();
  }

protected:
  void convert(const std::shared_ptr<const msensor::Scan3DI> &scan,
               sensors::PointCloud3 &out) override {
    const float vs = voxel_size_.load();
    pcl::VoxelGrid<msensor::Point3I> grid;
    grid.setInputCloud(scan->points);
    grid.setLeafSize(vs, vs, vs);
    auto filtered = std::make_shared<msensor::Scan3DI>();
    filtered->header = scan->header;
    grid.filter(*filtered->points);
    out = toProtobuf(filtered);
  }

private:
  std::atomic<float> voxel_size_{0.1f};
  sensors::SubSampledLidarStreamRequest request_;
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

grpc::ServerBidiReactor<sensors::SubSampledLidarStreamRequest,
                        sensors::PointCloud3> *
LidarServiceImpl::getSubSampledLidarScan(
    grpc::CallbackServerContext * /*context*/) {
  if (!lidar_) {
    return new UnavailableReactor<grpc::ServerBidiReactor<
        sensors::SubSampledLidarStreamRequest, sensors::PointCloud3>>();
  }
  auto *reactor = new SubSampledLidarReactor(lidar_->scans(), runner_);
  reactor->start();
  return reactor;
}
