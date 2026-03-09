#include "lidar_service.hh"
#include "conversions.hh"
#include "timing/timing.hh"
#include <atomic>
#include <pcl/filters/voxel_grid.h>

LidarServiceImpl::LidarServiceImpl(std::shared_ptr<msensor::ILidar> lidar)
    : lidar_(lidar) {}

// ---------------------------------------------------------------------------
// getLidarScan — server-streaming via WriteReactor
// ---------------------------------------------------------------------------

class LidarScanReactor : public grpc::ServerWriteReactor<sensors::PointCloud3> {
public:
  LidarScanReactor(std::shared_ptr<msensor::ILidar> lidar)
      : lidar_(lidar) {
    std::cout << "Start Lidar scan stream." << std::endl;
    NextWrite();
  }

  void OnWriteDone(bool ok) override {
    if (!ok) {
      Finish(grpc::Status::OK);
      return;
    }
    NextWrite();
  }

  void OnCancel() override {
    std::cout << "Ending Lidar scan stream." << std::endl;
  }

  void OnDone() override { delete this; }

private:
  void NextWrite() {
    const auto scan = lidar_->getScan();
    if (!scan) {
      NextWrite();
      return;
    }
    response_ = toGRPC(scan);
    StartWrite(&response_);
  }

  std::shared_ptr<msensor::ILidar> lidar_;
  sensors::PointCloud3 response_;
};

grpc::ServerWriteReactor<sensors::PointCloud3> *
LidarServiceImpl::getLidarScan(grpc::CallbackServerContext * /*context*/,
                               const sensors::LidarStreamRequest * /*request*/) {
  if (!lidar_) {
    auto *reactor = new LidarScanReactor(nullptr);
    reactor->Finish(grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                 "Lidar not available"));
    return reactor;
  }
  return new LidarScanReactor(lidar_);
}

// ---------------------------------------------------------------------------
// getSubSampledLidarScan — bidi streaming via BidiReactor
//
// Reads and writes are fully independent:
//   - OnReadDone:  updates the voxel size when the client sends a new value
//   - OnWriteDone: gets the next scan, filters it, and writes it back
// ---------------------------------------------------------------------------

class SubSampledLidarReactor
    : public grpc::ServerBidiReactor<sensors::SubSampledLidarStreamRequest,
                                     sensors::PointCloud3> {
public:
  SubSampledLidarReactor(std::shared_ptr<msensor::ILidar> lidar)
      : lidar_(lidar) {
    std::cout << "Start subsampled Lidar scan stream." << std::endl;
    StartRead(&request_);  // start listening for client messages
    NextWrite();            // start pushing scans immediately
  }

  void OnReadDone(bool ok) override {
    if (!ok)
      return; // client closed its half
    std::cout << "Received subsample request with voxel size: "
              << request_.voxel_size() << std::endl;
    voxel_size_.store(request_.voxel_size());
    StartRead(&request_); // keep listening
  }

  void OnWriteDone(bool ok) override {
    if (!ok) {
      Finish(grpc::Status::OK);
      return;
    }
    NextWrite();
  }

  void OnCancel() override {
    std::cout << "Ending subsampled Lidar scan stream." << std::endl;
  }

  void OnDone() override { delete this; }

private:
  void NextWrite() {
    const auto scan = lidar_->getScan();
    if (!scan) {
      // Scan failed - retry immediately (getScan has its own timeout)
      NextWrite();
      return;
    }
    float vs = voxel_size_.load();
    pcl::VoxelGrid<msensor::Point3I> grid;
    grid.setInputCloud(scan->points);
    grid.setLeafSize(vs, vs, vs);
    auto filtered = std::make_shared<msensor::Scan3DI>();
    filtered->timestamp = timing::getNowUs();
    grid.filter(*filtered->points);
    response_ = toGRPC(filtered);
    StartWrite(&response_);
  }

  std::shared_ptr<msensor::ILidar> lidar_;
  std::atomic<float> voxel_size_{0.1f};
  sensors::SubSampledLidarStreamRequest request_;
  sensors::PointCloud3 response_;
};

grpc::ServerBidiReactor<sensors::SubSampledLidarStreamRequest,
                        sensors::PointCloud3> *
LidarServiceImpl::getSubSampledLidarScan(
    grpc::CallbackServerContext * /*context*/) {
  if (!lidar_) {
    auto *reactor = new SubSampledLidarReactor(nullptr);
    reactor->Finish(grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                 "Lidar not available"));
    return reactor;
  }
  return new SubSampledLidarReactor(lidar_);
}
