

#include "sensors_service.hh"
#include "conversions.hh"

ScanService::ScanService(std::shared_ptr<IAdc> adc,
                         std::shared_ptr<ICamera> camera,
                         std::shared_ptr<IImu> imu,
                         std::shared_ptr<ILidar> lidar)
    : adc_(adc), camera_(camera), imu_(imu), lidar_(lidar) {}

grpc::Status
ScanService::getLidarScan(::grpc::ServerContext *context,
                          const ::sensors::SensorStreamRequest *request,
                          ::grpc::ServerWriter<sensors::PointCloud3> *writer) {

  static bool s_client_connected = false;

  if (!lidar_) {
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, "Lidar not available");
  }

  if (s_client_connected) {
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client per stream supported");
  }

  std::cout << "Start Lidar scan stream." << std::endl;

  s_client_connected = true;

  while (!context->IsCancelled()) {
    if (const auto scan = lidar_->getScan()) {
      writer->Write(toGRPC(scan));
    }
  }
  std::cout << "Ending Lidar scan stream." << std::endl;
  s_client_connected = false;

  return ::grpc::Status::OK;
}

::grpc::Status
ScanService::getImuData(::grpc::ServerContext *context,
                        const ::sensors::SensorStreamRequest *request,
                        ::grpc::ServerWriter<sensors::IMUData> *writer) {

  static bool s_client_connected = false;

  if (!imu_) {
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, "IMU not available");
  }

  if (s_client_connected) {
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client per stream supported");
  }

  std::cout << "Start IMU data stream." << std::endl;

  s_client_connected = true;

  while (!context->IsCancelled()) {
    if (const auto imu_data = imu_->getImuData()) {
      writer->Write(toGRPC(*imu_data));
    }
  }

  std::cout << "Ending IMU data stream." << std::endl;
  s_client_connected = false;

  return ::grpc::Status::OK;
}

::grpc::Status ScanService::getCameraFrame(
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

    CameraFrame frame;
    if (camera_->read(frame)) {
      writer->Write(toGRPC(frame));
    }
  }

  std::cout << "Ending camera stream." << std::endl;
  s_client_connected = false;

  return ::grpc::Status::OK;
}

::grpc::Status ScanService::getAdcData(::grpc::ServerContext *context,
                                       const ::sensors::AdcDataRequest *request,
                                       ::sensors::AdcData *response) {
  if (!adc_) {
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, "ADC not available");
  }

  const auto adc_value = adc_->readSingleEnded();
  if (!adc_value) {
    return grpc::Status(grpc::StatusCode::UNKNOWN, "ADC read failure");
  }

  response->set_sample(adc_value->voltage);
  response->set_timestamp(adc_value->timestamp);
  return ::grpc::Status::OK;
}
