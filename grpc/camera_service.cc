
#include "camera_service.hh"
#include "conversions.hh"

CameraServiceImpl::CameraServiceImpl(std::shared_ptr<msensor::ICamera> camera)
    : camera_(camera) {}

::grpc::Status CameraServiceImpl::getCameraFrame(
    ::grpc::ServerContext *context,
    const ::sensors::CameraStreamRequest *request,
    ::grpc::ServerWriter<::sensors::CameraStreamReply> *writer) {

  static bool s_client_connected = false;

  if (!camera_) {
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, "Camera not available");
  }

  if (s_client_connected) {
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client per stream supported");
  }

  std::cout << "Start camera stream." << std::endl;

  s_client_connected = true;

  while (!context->IsCancelled()) {

    msensor::CameraFrame frame;
    if (camera_->read(frame)) {
      writer->Write(toGRPC(frame));
    }
  }

  std::cout << "Ending camera stream." << std::endl;
  s_client_connected = false;

  return ::grpc::Status::OK;
}
