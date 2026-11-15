#include "imu_service.hh"
#include "msensor/conversions/lidar_imu_conversions.hh"
#include "sensor_stream.hh"
#include <utility>

namespace msensor {

ImuServiceImpl::ImuServiceImpl(std::shared_ptr<msensor::IImu> imu)
    : imu_(std::move(imu)) {}

grpc::ServerWriteReactor<sensors::IMUData> *
ImuServiceImpl::getImuData(grpc::CallbackServerContext * /*context*/,
                           const sensors::ImuStreamRequest *request) {
  return msensor::makeStreamReactor<msensor::IMUData, sensors::IMUData>(
      imu_, in_use_, "IMU not available", &msensor::IImu::setImuCallback,
      [](const msensor::IMUData &sample, sensors::IMUData &out) {
        toProtobuf(sample, out);
      },
      "IMU data", request->max_queued_samples(), msensor::kImuQueueLimits);
}

} // namespace msensor
