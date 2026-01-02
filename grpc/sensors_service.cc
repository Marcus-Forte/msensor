#include <opencv2/imgcodecs.hpp>

#include "sensors_service.hh"
#include "timing/timing.hh"

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

    const auto scan = lidar_->getScan();
    if (!scan) {
      continue;
    }
    sensors::PointCloud3 point_cloud;

    point_cloud.set_timestamp(scan->timestamp);
    for (const auto &point : *scan->points) {
      auto pt = point_cloud.add_points();
      pt->set_x(point.x);
      pt->set_y(point.y);
      pt->set_z(point.z);
      pt->set_intensity(point.intensity);
    }
    writer->Write(point_cloud);
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

    const auto imu_data = imu_->getImuData();
    if (!imu_data) {
      continue;
    }

    sensors::IMUData grpc_data;
    grpc_data.set_ax(imu_data->ax);
    grpc_data.set_ay(imu_data->ay);
    grpc_data.set_az(imu_data->az);
    grpc_data.set_gx(imu_data->gx);
    grpc_data.set_gy(imu_data->gy);
    grpc_data.set_gz(imu_data->gz);
    grpc_data.set_timestamp(imu_data->timestamp);
    writer->Write(grpc_data);
  }

  std::cout << "Ending IMU data stream." << std::endl;
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
    if (!camera_->read(frame)) {
      std::cerr << "Failed to read frame from camera." << std::endl;
      continue;
    }

    sensors::CameraStreamReply reply;
    reply.set_width(frame.mat.cols);
    reply.set_height(frame.mat.rows);

    // Set encoding based on OpenCV Mat type
    if (frame.mat.channels() == 3) {
      reply.set_encoding(sensors::CameraEncoding::BGR8);
    } else if (frame.mat.channels() == 1) {
      reply.set_encoding(sensors::CameraEncoding::GRAY8);
    } else {
      reply.set_encoding(sensors::CameraEncoding::UNKNOWN);
    }

    // Set timestamp
    reply.set_timestamp(frame.timestamp);

    std::vector<uchar> jpeg_buffer;
    const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 85};
    if (!cv::imencode(".jpg", frame.mat, jpeg_buffer, jpeg_params)) {
      std::cerr << "Failed to encode frame as JPEG." << std::endl;

      continue;
    }

    reply.set_encoding(sensors::CameraEncoding::MJPEG);
    reply.set_image_data(jpeg_buffer.data(), jpeg_buffer.size());

    // std::cout << "Sent: " << reply.image_data().size() << " bytes"
    //           << " Width: " << reply.width() << " Height: " <<
    //           reply.height()
    //           << " Encoding: " << reply.encoding() << std::endl;

    writer->Write(reply);
  }

  std::cout << "Ending camera stream." << std::endl;
  s_client_connected = false;
  return ::grpc::Status::OK;
}
