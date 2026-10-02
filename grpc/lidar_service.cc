#include "lidar_service.hh"
#include "msensor/conversions/conversions.hh"
#include "sensor_stream.hh"
#include <utility>

LidarServiceImpl::LidarServiceImpl(std::shared_ptr<msensor::ILidar> lidar)
    : lidar_(std::move(lidar)) {}

grpc::ServerWriteReactor<sensors::PointCloud3> *LidarServiceImpl::getLidarScan(
    grpc::CallbackServerContext * /*context*/,
    const sensors::LidarStreamRequest * /*request*/) {
  if (!lidar_) {
    return new msensor::ErrorReactor<sensors::PointCloud3>(
        {grpc::StatusCode::UNAVAILABLE, "Lidar not available"});
  }
  if (in_use_.exchange(true)) {
    return new msensor::ErrorReactor<sensors::PointCloud3>(
        {grpc::StatusCode::RESOURCE_EXHAUSTED,
         "Only one client per stream supported"});
  }
  return new msensor::SensorStreamReactor<msensor::Scan3DI,
                                          sensors::PointCloud3>(
      [this](auto callback) { lidar_->setScanCallback(std::move(callback)); },
      [](const msensor::Scan3DI &scan, sensors::PointCloud3 &out) {
        out = toProtobuf(scan);
      },
      in_use_, "Lidar scan");
}
