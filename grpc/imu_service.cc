#include "imu_service.hh"
#include "msensor/conversions/lidar_imu_conversions.hh"
#include "sensor_stream.hh"
#include <utility>

namespace msensor {

namespace {
Sensors onlySensor(std::shared_ptr<IImu> sensor) {
  Sensors sensors;
  sensors.imu = std::move(sensor);
  return sensors;
}
} // namespace

ImuServiceImpl::ImuServiceImpl(std::shared_ptr<msensor::IImu> imu)
    : ImuServiceImpl(std::make_shared<SensorBus>(onlySensor(std::move(imu))),
                     kDefaultMaxClientsPerStream) {}

ImuServiceImpl::ImuServiceImpl(std::shared_ptr<SensorBus> bus,
                               size_t max_clients)
    : bus_(bus ? std::move(bus) : std::make_shared<SensorBus>(Sensors{})),
      limit_(max_clients) {}

grpc::ServerWriteReactor<sensors::IMUData> *
ImuServiceImpl::getImuData(grpc::CallbackServerContext * /*context*/,
                           const sensors::ImuStreamRequest *request) {
  return msensor::makeStreamReactor<msensor::IMUData, sensors::IMUData>(
      bus_->imu(), limit_, "IMU not available",
      [](const msensor::IMUData &sample, sensors::IMUData &out) {
        toProtobuf(sample, out);
      },
      "IMU data", request->max_queued_samples(), msensor::kImuQueueLimits);
}

} // namespace msensor
