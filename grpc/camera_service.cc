#include "camera_service.hh"
#include "msensor/conversions/conversions.hh"
#include "sensor_stream.hh"
#include <utility>

CameraServiceImpl::CameraServiceImpl(std::shared_ptr<msensor::ICamera> camera)
    : camera_(std::move(camera)) {}

grpc::ServerWriteReactor<sensors::CameraStreamReply> *
CameraServiceImpl::getCameraFrame(
    grpc::CallbackServerContext * /*context*/,
    const sensors::CameraStreamRequest * /*request*/) {
  return msensor::makeStreamReactor<msensor::CameraFrame,
                                    sensors::CameraStreamReply>(
      camera_, in_use_, "Camera not available",
      &msensor::ICamera::setFrameCallback,
      [](const msensor::CameraFrame &frame, sensors::CameraStreamReply &out) {
        toProtobuf(frame, out);
      },
      "camera");
}
