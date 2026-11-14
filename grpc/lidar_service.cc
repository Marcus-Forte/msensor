#include "lidar_service.hh"
#include "msensor/conversions/lidar_imu_conversions.hh"
#include "sensor_stream.hh"
#include <utility>

namespace msensor {

LidarServiceImpl::LidarServiceImpl(std::shared_ptr<msensor::ILidar> lidar)
    : lidar_(std::move(lidar)) {}

grpc::ServerWriteReactor<sensors::PointCloud3> *LidarServiceImpl::getLidarScan(
    grpc::CallbackServerContext * /*context*/,
    const sensors::LidarStreamRequest * /*request*/) {
  return msensor::makeStreamReactor<msensor::Scan3DI, sensors::PointCloud3>(
      lidar_, in_use_, "Lidar not available", &msensor::ILidar::setScanCallback,
      [](const msensor::Scan3DI &scan, sensors::PointCloud3 &out) {
        toProtobuf(scan, out);
      },
      "Lidar scan");
}

} // namespace msensor
