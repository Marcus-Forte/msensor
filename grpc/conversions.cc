
#include <opencv2/imgcodecs.hpp>

#include "conversions.hh"

std::shared_ptr<msensor::Scan3DI> fromGRPC(const sensors::PointCloud3 &msg) {

  auto scan = std::make_shared<msensor::Scan3DI>();

  scan->points->reserve(msg.points_size());
  for (const auto &pt : msg.points()) {
    scan->points->emplace_back(pt.x(), pt.y(), pt.z());
  }
  scan->timestamp = msg.timestamp();

  return scan;
}

sensors::PointCloud3 toGRPC(const std::shared_ptr<msensor::Scan3DI> &scan) {
  sensors::PointCloud3 point_cloud;

  point_cloud.set_timestamp(scan->timestamp);
  for (const auto &point : *scan->points) {
    auto pt = point_cloud.add_points();
    pt->set_x(point.x);
    pt->set_y(point.y);
    pt->set_z(point.z);
    pt->set_intensity(point.intensity);
  }

  return point_cloud;
}

msensor::IMUData fromGRPC(const sensors::IMUData &msg) {
  msensor::IMUData imu_data;
  imu_data.ax = msg.ax();
  imu_data.ay = msg.ay();
  imu_data.az = msg.az();
  imu_data.gx = msg.gx();
  imu_data.gy = msg.gy();
  imu_data.gz = msg.gz();
  imu_data.timestamp = msg.timestamp();
  return imu_data;
}

sensors::IMUData toGRPC(msensor::IMUData imu_data) {
  sensors::IMUData grpc_data;
  grpc_data.set_ax(imu_data.ax);
  grpc_data.set_ay(imu_data.ay);
  grpc_data.set_az(imu_data.az);
  grpc_data.set_gx(imu_data.gx);
  grpc_data.set_gy(imu_data.gy);
  grpc_data.set_gz(imu_data.gz);
  grpc_data.set_timestamp(imu_data.timestamp);
  return grpc_data;
}

sensors::CameraStreamReply toGRPC(const msensor::CameraFrame &frame,
                                  int quality) {
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

  reply.set_timestamp(frame.timestamp);

  static std::vector<uchar> jpeg_buffer;
  const std::vector<int> jpeg_params{cv::IMWRITE_JPEG_QUALITY, quality};
  if (!cv::imencode(".jpg", frame.mat, jpeg_buffer, jpeg_params)) {
    // std::cerr << "Failed to encode frame as JPEG." << std::endl;
  }

  reply.set_encoding(sensors::CameraEncoding::MJPEG);
  reply.set_image_data(jpeg_buffer.data(), jpeg_buffer.size());

  return reply;
}