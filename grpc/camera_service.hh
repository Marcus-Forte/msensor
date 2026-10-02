#pragma once

#include <atomic>
#include <memory>

#include "camera.grpc.pb.h"
#include "msensor/interface/ICamera.hh"

/**
 * @brief Implements the Camera gRPC service.
 *
 * One client per stream: a second concurrent stream is rejected with
 * RESOURCE_EXHAUSTED. Frames are pushed by the camera's callback, so the
 * service holds no thread waiting for data.
 */
class CameraServiceImpl : public sensors::CameraService::CallbackService {
public:
  explicit CameraServiceImpl(std::shared_ptr<msensor::ICamera> camera);

  grpc::ServerWriteReactor<sensors::CameraStreamReply> *
  getCameraFrame(grpc::CallbackServerContext *context,
                 const sensors::CameraStreamRequest *request) override;

private:
  std::shared_ptr<msensor::ICamera> camera_;
  std::atomic<bool> in_use_{false};
};
