
#include <opencv2/imgcodecs.hpp>

#include "conversions.hh"

std::shared_ptr<msensor::Scan3DI> fromProtobuf(const sensors::PointCloud3 &msg) {

  auto scan = std::make_shared<msensor::Scan3DI>();
  if (msg.x_size() != msg.y_size() || msg.x_size() != msg.z_size() || msg.x_size() != msg.intensity_size()) {
    return scan; 
  }

  const float *x_data = msg.x().data();
  const float *y_data = msg.y().data();
  const float *z_data = msg.z().data();
  const uint32_t *intensity_data = msg.intensity().data();

  scan->points->resize(msg.x_size());

  for(int i = 0; i < msg.x_size(); ++i) {
    (*scan->points)[i].x = x_data[i];
    (*scan->points)[i].y = y_data[i];
    (*scan->points)[i].z = z_data[i];
    (*scan->points)[i].intensity = intensity_data[i];
  }

  scan->header.timestamp = msg.header().timestamp();

  return scan;
}

sensors::PointCloud3 toProtobuf(const std::shared_ptr<msensor::Scan3DI> &scan) {
  sensors::PointCloud3 point_cloud;

  if (!scan || !scan->points) {
    return point_cloud;
  }

  point_cloud.mutable_header()->set_timestamp(scan->header.timestamp);
  point_cloud.mutable_header()->set_sequence_number(scan->header.sequence_number);

  auto *x = point_cloud.mutable_x();
  auto *y = point_cloud.mutable_y();
  auto *z = point_cloud.mutable_z();
  auto *intensity = point_cloud.mutable_intensity();

  const auto point_count = static_cast<int>(scan->points->size());
  x->Reserve(point_count);
  y->Reserve(point_count);
  z->Reserve(point_count);
  intensity->Reserve(point_count);

  for (const auto &point : scan->points->points) {
    x->Add(point.x);
    y->Add(point.y);
    z->Add(point.z);
    intensity->Add(static_cast<uint32_t>(point.intensity));
  }

  return point_cloud;
}

msensor::IMUData fromProtobuf(const sensors::IMUData &msg) {
  msensor::IMUData imu_data;
  imu_data.header.timestamp = msg.header().timestamp();
  imu_data.ax = msg.ax();
  imu_data.ay = msg.ay();
  imu_data.az = msg.az();
  imu_data.gx = msg.gx();
  imu_data.gy = msg.gy();
  imu_data.gz = msg.gz();
  return imu_data;
}

sensors::IMUData toProtobuf(msensor::IMUData imu_data) {
  sensors::IMUData grpc_data;
  grpc_data.set_ax(imu_data.ax);
  grpc_data.set_ay(imu_data.ay);
  grpc_data.set_az(imu_data.az);
  grpc_data.set_gx(imu_data.gx);
  grpc_data.set_gy(imu_data.gy);
  grpc_data.set_gz(imu_data.gz);
  grpc_data.mutable_header()->set_timestamp(imu_data.header.timestamp);
  grpc_data.mutable_header()->set_sequence_number(imu_data.header.sequence_number);
  return grpc_data;
}

sensors::CameraStreamReply toProtobuf(const msensor::CameraFrame &frame,
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

  reply.mutable_header()->set_timestamp(frame.header.timestamp);
  reply.mutable_header()->set_sequence_number(frame.header.sequence_number);

  static std::vector<uchar> jpeg_buffer;
  const std::vector<int> jpeg_params{cv::IMWRITE_JPEG_QUALITY, quality};
  if (!cv::imencode(".jpg", frame.mat, jpeg_buffer, jpeg_params)) {
    // std::cerr << "Failed to encode frame as JPEG." << std::endl;
  }

  reply.set_encoding(sensors::CameraEncoding::MJPEG);
  reply.set_image_data(jpeg_buffer.data(), jpeg_buffer.size());

  return reply;
}