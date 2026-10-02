#include "imu_service.hh"
#include "msensor/conversions/conversions.hh"
#include "sensor_stream.hh"
#include <utility>

ImuServiceImpl::ImuServiceImpl(std::shared_ptr<msensor::IImu> imu)
    : imu_(std::move(imu)) {}

grpc::ServerWriteReactor<sensors::IMUData> *ImuServiceImpl::getImuData(
    grpc::CallbackServerContext * /*context*/,
    const sensors::ImuStreamRequest * /*request*/) {
  if (!imu_) {
    return new msensor::ErrorReactor<sensors::IMUData>(
        {grpc::StatusCode::UNAVAILABLE, "IMU not available"});
  }
  if (in_use_.exchange(true)) {
    return new msensor::ErrorReactor<sensors::IMUData>(
        {grpc::StatusCode::RESOURCE_EXHAUSTED,
         "Only one client per stream supported"});
  }
  return new msensor::SensorStreamReactor<msensor::IMUData, sensors::IMUData>(
      [this](auto callback) { imu_->setImuCallback(std::move(callback)); },
      [](const msensor::IMUData &sample, sensors::IMUData &out) {
        toProtobuf(sample, out);
      },
      in_use_, "IMU data");
}
