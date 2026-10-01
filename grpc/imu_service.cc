#include "imu_service.hh"
#include "msensor/conversions/conversions.hh"
#include <iostream>

namespace {
constexpr auto kPollTimeout = std::chrono::milliseconds(100);
// About one second of samples at 200 Hz.
constexpr std::size_t kMaxBufferedSamples = 200;
} // namespace

ImuServiceImpl::ImuServiceImpl(std::shared_ptr<msensor::IImu> imu)
    : imu_(imu) {}

::grpc::Status
ImuServiceImpl::getImuData(::grpc::ServerContext *context,
                           const ::sensors::ImuStreamRequest *request,
                           ::grpc::ServerWriter<sensors::IMUData> *writer) {

  if (!imu_) {
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, "IMU not available");
  }

  std::cout << "Start IMU data stream." << std::endl;

  auto sub = imu_->imu().subscribe(
      msensor::SubscribePolicy::bounded(kMaxBufferedSamples));

  // Blocks on the subscription instead of polling. The timeout only bounds
  // how long it takes to notice that the client went away.
  while (!context->IsCancelled()) {
    if (const auto imu_data = sub->waitPop({}, kPollTimeout)) {
      if (!writer->Write(toProtobuf(*imu_data))) {
        break;
      }
    }
  }

  std::cout << "Ending IMU data stream." << std::endl;

  return ::grpc::Status::OK;
}
