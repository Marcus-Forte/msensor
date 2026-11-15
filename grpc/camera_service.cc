#include "camera_service.hh"
#include "msensor/conversions/camera_conversions.hh"
#include "sensor_stream.hh"
#include <stdexcept>
#include <string>
#include <utility>

namespace msensor {

namespace {
Sensors onlySensor(std::shared_ptr<ICamera> sensor) {
  Sensors sensors;
  sensors.camera = std::move(sensor);
  return sensors;
}
} // namespace

CameraServiceImpl::CameraServiceImpl(std::shared_ptr<msensor::ICamera> camera,
                                     int jpeg_quality)
    : CameraServiceImpl(
          std::make_shared<SensorBus>(onlySensor(std::move(camera))),
          jpeg_quality, kDefaultMaxClientsPerStream) {}

CameraServiceImpl::CameraServiceImpl(std::shared_ptr<SensorBus> bus,
                                     int jpeg_quality, size_t max_clients)
    : bus_(bus ? std::move(bus) : std::make_shared<SensorBus>(Sensors{})),
      jpeg_quality_(jpeg_quality), limit_(max_clients) {
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
      bus_->camera(), limit_, "Camera not available",
      [quality = jpeg_quality_](const msensor::CameraFrame &frame,
                                sensors::CameraStreamReply &out) {
        toProtobuf(frame, out, quality);
      },
      "camera", request->max_queued_samples(), msensor::kCameraQueueLimits);
}

} // namespace msensor
