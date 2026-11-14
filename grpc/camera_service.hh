#pragma once

#include <atomic>
#include <memory>

#include "camera.grpc.pb.h"
#include "msensor/interface/ICamera.hh"

namespace msensor {

/**
 * @brief Implements the Camera gRPC service.
 *
 * One client per stream: a second concurrent stream is rejected with
 * RESOURCE_EXHAUSTED. Frames are pushed by the camera's callback, so the
 * service holds no thread waiting for data.
 */
class CameraServiceImpl : public sensors::CameraService::CallbackService {
public:
  /// @param jpeg_quality JPEG quality [0-100] used when raw frames are encoded.
  /// Throws std::invalid_argument if it is outside that range.
  explicit CameraServiceImpl(std::shared_ptr<msensor::ICamera> camera,
                             int jpeg_quality = 85);

  grpc::ServerWriteReactor<sensors::CameraStreamReply> *
  getCameraFrame(grpc::CallbackServerContext *context,
                 const sensors::CameraStreamRequest *request) override;

private:
  std::shared_ptr<msensor::ICamera> camera_;
  int jpeg_quality_;
  std::atomic<bool> in_use_{false};
};

} // namespace msensor

#ifndef MSENSOR_NO_LEGACY_NAMES
// Legacy global names, for downstream code. Define MSENSOR_NO_LEGACY_NAMES to
// drop them.
using msensor::CameraServiceImpl;
#endif
