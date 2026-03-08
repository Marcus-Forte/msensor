#pragma once

#include "camera.grpc.pb.h"
#include "interface/ICamera.hh"

/**
 * @brief Implements the Camera gRPC service.
 */
class CameraServiceImpl : public sensors::CameraService::Service {
public:
  CameraServiceImpl(std::shared_ptr<msensor::ICamera> camera);

  ::grpc::Status getCameraFrame(
      ::grpc::ServerContext *context,
      const ::sensors::CameraStreamRequest *request,
      ::grpc::ServerWriter<::sensors::CameraStreamReply> *writer) override;

private:
  std::shared_ptr<msensor::ICamera> camera_;
};
