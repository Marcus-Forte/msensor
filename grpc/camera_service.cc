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
  if (!camera_) {
    return new msensor::ErrorReactor<sensors::CameraStreamReply>(
        {grpc::StatusCode::UNAVAILABLE, "Camera not available"});
  }
  if (in_use_.exchange(true)) {
    return new msensor::ErrorReactor<sensors::CameraStreamReply>(
        {grpc::StatusCode::RESOURCE_EXHAUSTED,
         "Only one client per stream supported"});
  }
  return new msensor::SensorStreamReactor<msensor::CameraFrame,
                                          sensors::CameraStreamReply>(
      [this](auto callback) { camera_->setFrameCallback(std::move(callback)); },
      [](const msensor::CameraFrame &frame, sensors::CameraStreamReply &out) {
        toProtobuf(frame, out);
      },
      in_use_, "camera");
}
