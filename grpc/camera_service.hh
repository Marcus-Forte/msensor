#pragma once

#include <cstddef>
#include <memory>

#include "camera.grpc.pb.h"
#include "msensor/bus/sensor_bus.hh"
#include "msensor/interface/ICamera.hh"
#include "sensor_stream.hh"

namespace msensor {

/**
 * @brief Implements the Camera gRPC service.
 *
 * Any number of clients up to `max_clients` may stream at once; one more is
 * rejected with RESOURCE_EXHAUSTED. Each is a subscriber of the camera's bus
 * topic. Frames are pushed by the camera's callback, so the
 * service holds no thread waiting for data.
 */
class CameraServiceImpl : public sensors::CameraService::CallbackService {
public:
  /// @param jpeg_quality JPEG quality [0-100] used when raw frames are encoded.
  /// Throws std::invalid_argument if it is outside that range.
  explicit CameraServiceImpl(std::shared_ptr<msensor::ICamera> camera,
                             int jpeg_quality = 85);

  /// Serves the camera topic of a shared bus to at most `max_clients` clients
  /// at once. Throws std::invalid_argument like the constructor above.
  CameraServiceImpl(std::shared_ptr<SensorBus> bus, int jpeg_quality,
                    size_t max_clients);

  grpc::ServerWriteReactor<sensors::CameraStreamReply> *
  getCameraFrame(grpc::CallbackServerContext *context,
                 const sensors::CameraStreamRequest *request) override;

private:
  std::shared_ptr<SensorBus> bus_;
  int jpeg_quality_;
  ClientLimit limit_;
};

} // namespace msensor

#ifndef MSENSOR_NO_LEGACY_NAMES
// Legacy global names, for downstream code. Define MSENSOR_NO_LEGACY_NAMES to
// drop them.
using msensor::CameraServiceImpl;
#endif
