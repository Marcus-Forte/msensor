#include "camera_service.hh"
#include "msensor/conversions/camera_conversions.hh"
#include "sensor_stream.hh"
#include <stdexcept>
#include <string>
#include <utility>

namespace msensor {

CameraServiceImpl::CameraServiceImpl(std::shared_ptr<msensor::ICamera> camera,
                                     int jpeg_quality)
    : camera_(std::move(camera)), jpeg_quality_(jpeg_quality) {
  if (jpeg_quality < 0 || jpeg_quality > 100) {
    throw std::invalid_argument("JPEG quality must be in [0, 100], got " +
                                std::to_string(jpeg_quality));
  }
}

grpc::ServerWriteReactor<sensors::CameraStreamReply> *
CameraServiceImpl::getCameraFrame(grpc::CallbackServerContext * /*context*/,
                                  const sensors::CameraStreamRequest *request) {
  return msensor::makeStreamReactor<msensor::CameraFrame,
                                    sensors::CameraStreamReply>(
      camera_, in_use_, "Camera not available",
      &msensor::ICamera::setFrameCallback,
      [quality = jpeg_quality_](const msensor::CameraFrame &frame,
                                sensors::CameraStreamReply &out) {
        toProtobuf(frame, out, quality);
      },
      "camera", request->max_queued_samples(), msensor::kCameraQueueLimits);
}

} // namespace msensor
