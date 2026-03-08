
#include "imu_service.hh"
#include "conversions.hh"

ImuServiceImpl::ImuServiceImpl(std::shared_ptr<msensor::IImu> imu)
    : imu_(imu) {}

::grpc::Status
ImuServiceImpl::getImuData(::grpc::ServerContext *context,
                           const ::sensors::ImuStreamRequest *request,
                           ::grpc::ServerWriter<sensors::IMUData> *writer) {

  static bool s_client_connected = false;

  if (!imu_) {
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, "IMU not available");
  }

  if (s_client_connected) {
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client per stream supported");
  }

  std::cout << "Start IMU data stream." << std::endl;

  s_client_connected = true;

  while (!context->IsCancelled()) {
    if (const auto imu_data = imu_->getImuData()) {
      writer->Write(toGRPC(*imu_data));
    }
  }

  std::cout << "Ending IMU data stream." << std::endl;
  s_client_connected = false;

  return ::grpc::Status::OK;
}
