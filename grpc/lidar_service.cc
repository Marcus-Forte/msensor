#include "lidar_service.hh"
#include "msensor/conversions/lidar_imu_conversions.hh"
#include "sensor_stream.hh"
#include <utility>

namespace msensor {

namespace {
Sensors onlySensor(std::shared_ptr<ILidar> sensor) {
  Sensors sensors;
  sensors.lidar = std::move(sensor);
  return sensors;
}
} // namespace

LidarServiceImpl::LidarServiceImpl(std::shared_ptr<msensor::ILidar> lidar)
    : LidarServiceImpl(
          std::make_shared<SensorBus>(onlySensor(std::move(lidar))),
          kDefaultMaxClientsPerStream) {}

LidarServiceImpl::LidarServiceImpl(std::shared_ptr<SensorBus> bus,
                                   size_t max_clients)
    : bus_(bus ? std::move(bus) : std::make_shared<SensorBus>(Sensors{})),
      limit_(max_clients) {}

grpc::ServerWriteReactor<sensors::PointCloud3> *
LidarServiceImpl::getLidarScan(grpc::CallbackServerContext * /*context*/,
                               const sensors::LidarStreamRequest *request) {
  return msensor::makeStreamReactor<msensor::Scan3DI, sensors::PointCloud3>(
      bus_->lidar(), limit_, "Lidar not available",
      [](const msensor::Scan3DI &scan, sensors::PointCloud3 &out) {
        toProtobuf(scan, out);
      },
      "Lidar scan", request->max_queued_samples(), msensor::kLidarQueueLimits);
}

} // namespace msensor
