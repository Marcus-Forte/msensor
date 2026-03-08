#pragma once

#include "imu.grpc.pb.h"
#include "interface/IImu.hh"

/**
 * @brief Implements the IMU gRPC service.
 */
class ImuServiceImpl : public sensors::ImuService::Service {
public:
  ImuServiceImpl(std::shared_ptr<msensor::IImu> imu);

  ::grpc::Status
  getImuData(::grpc::ServerContext *context,
             const ::sensors::ImuStreamRequest *request,
             ::grpc::ServerWriter<sensors::IMUData> *writer) override;

private:
  std::shared_ptr<msensor::IImu> imu_;
};
