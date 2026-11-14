
#include <opencv2/imgcodecs.hpp>

#include "msensor/conversions/conversions.hh"

#include <vector>

msensor::Scan3DI fromProtobuf(const sensors::PointCloud3 &msg) {

  msensor::Scan3DI scan;
  if (msg.x_size() != msg.y_size() || msg.x_size() != msg.z_size() ||
      msg.x_size() != msg.intensity_size()) {
    return scan;
  }

  const float *x_data = msg.x().data();
  const float *y_data = msg.y().data();
  const float *z_data = msg.z().data();
  const uint32_t *intensity_data = msg.intensity().data();

  scan.points.resize(msg.x_size());

  for (int i = 0; i < msg.x_size(); ++i) {
    scan.points[i].x = x_data[i];
    scan.points[i].y = y_data[i];
    scan.points[i].z = z_data[i];
    scan.points[i].intensity = intensity_data[i];
  }

  scan.header.timestamp = msg.header().timestamp();
  scan.header.sequence_number = msg.header().sequence_number();

  return scan;
}

void toProtobuf(const msensor::Scan3DI &scan,
                sensors::PointCloud3 &point_cloud) {
  point_cloud.Clear();

  auto *header = point_cloud.mutable_header();
  header->set_timestamp(scan.header.timestamp);
  header->set_sequence_number(scan.header.sequence_number);

  const auto point_count = static_cast<int>(scan.points.size());
  auto *x = point_cloud.mutable_x();
  auto *y = point_cloud.mutable_y();
  auto *z = point_cloud.mutable_z();
  auto *intensity = point_cloud.mutable_intensity();
  // Clear() keeps the capacity, so a reused message allocates nothing here.
  x->Reserve(point_count);
  y->Reserve(point_count);
  z->Reserve(point_count);
  intensity->Reserve(point_count);

  // Fill the columns through raw pointers: one size update per field instead
  // of one per element lets the compiler vectorise the AoS -> SoA transpose.
  float *const x_out = x->AddNAlreadyReserved(point_count);
  float *const y_out = y->AddNAlreadyReserved(point_count);
  float *const z_out = z->AddNAlreadyReserved(point_count);
  uint32_t *const intensity_out = intensity->AddNAlreadyReserved(point_count);
  const msensor::Point3I *const points = scan.points.data();
  for (int i = 0; i < point_count; ++i) {
    x_out[i] = points[i].x;
    y_out[i] = points[i].y;
    z_out[i] = points[i].z;
    intensity_out[i] = static_cast<uint32_t>(points[i].intensity);
  }
}

msensor::IMUData fromProtobuf(const sensors::IMUData &msg) {
  msensor::IMUData imu_data;
  imu_data.header.timestamp = msg.header().timestamp();
  imu_data.header.sequence_number = msg.header().sequence_number();
  imu_data.ax = msg.ax();
  imu_data.ay = msg.ay();
  imu_data.az = msg.az();
  imu_data.gx = msg.gx();
  imu_data.gy = msg.gy();
  imu_data.gz = msg.gz();
  return imu_data;
}

void toProtobuf(const msensor::IMUData &imu_data,
                sensors::IMUData &grpc_data) {
  grpc_data.Clear();
  grpc_data.set_ax(imu_data.ax);
  grpc_data.set_ay(imu_data.ay);
  grpc_data.set_az(imu_data.az);
  grpc_data.set_gx(imu_data.gx);
  grpc_data.set_gy(imu_data.gy);
  grpc_data.set_gz(imu_data.gz);
  grpc_data.mutable_header()->set_timestamp(imu_data.header.timestamp);
  grpc_data.mutable_header()->set_sequence_number(
      imu_data.header.sequence_number);
}

namespace {

/// Reads the image size from the first start-of-frame marker of a JPEG.
/// Returns false if the stream is not a JPEG or ends before a SOF segment.
bool readJpegSize(const uchar *data, size_t size, int &width, int &height) {
  if (size < 4 || data[0] != 0xFF || data[1] != 0xD8) {
    return false;
  }
  size_t pos = 2;
  while (pos + 4 <= size) {
    if (data[pos] != 0xFF) {
      return false; // not at a marker
    }
    const uchar marker = data[pos + 1];
    if (marker == 0xFF) { // fill byte
      ++pos;
      continue;
    }
    if (marker == 0x00 || marker == 0x01 || (marker >= 0xD0 && marker <= 0xD8)) {
      pos += 2; // stuffed byte, TEM, RSTn, SOI: no length field
      continue;
    }
    if (marker == 0xD9 || marker == 0xDA) {
      return false; // EOI / start of scan reached without a SOF
    }
    const size_t length = (size_t{data[pos + 2]} << 8) | data[pos + 3];
    if (length < 2) {
      return false;
    }
    const bool is_sof = marker >= 0xC0 && marker <= 0xCF && marker != 0xC4 &&
                        marker != 0xC8 && marker != 0xCC;
    if (is_sof) {
      // length(2) precision(1) height(2) width(2)
      if (length < 7 || pos + 9 > size) {
        return false;
      }
      height = (data[pos + 5] << 8) | data[pos + 6];
      width = (data[pos + 7] << 8) | data[pos + 8];
      return true;
    }
    pos += 2 + length;
  }
  return false;
}

} // namespace

void toProtobuf(const msensor::CameraFrame &frame,
                sensors::CameraStreamReply &reply, int quality) {
  reply.Clear();
  reply.mutable_header()->set_timestamp(frame.header.timestamp);
  reply.mutable_header()->set_sequence_number(frame.header.sequence_number);

  if (frame.encoding == msensor::CameraFrameEncoding::Jpeg) {
    // Already JPEG: pass the bytes through without decoding or re-encoding.
    const size_t size = frame.mat.total() * frame.mat.elemSize();
    if (frame.mat.empty() || size == 0) {
      reply.set_encoding(sensors::CameraEncoding::UNKNOWN);
      return;
    }
    int width = 0;
    int height = 0;
    if (frame.mat.isContinuous()) {
      const auto *bytes = frame.mat.data;
      if (!readJpegSize(bytes, size, width, height)) {
        width = height = 0;
      }
      reply.set_image_data(bytes, size);
    } else {
      // Rows are separated by padding: gather the bytes first.
      const cv::Mat packed = frame.mat.clone(); // clone() yields a continuous Mat
      if (!readJpegSize(packed.data, size, width, height)) {
        width = height = 0;
      }
      reply.set_image_data(packed.data, size);
    }
    reply.set_width(width);
    reply.set_height(height);
    reply.set_encoding(sensors::CameraEncoding::MJPEG);
    return;
  }

  reply.set_width(frame.mat.cols);
  reply.set_height(frame.mat.rows);

  // One encode buffer per thread: it keeps its capacity between frames, and
  // concurrent conversions (e.g. two servers in one process) cannot race.
  thread_local std::vector<uchar> jpeg_buffer;
  const std::vector<int> jpeg_params{cv::IMWRITE_JPEG_QUALITY, quality};
  if (frame.mat.empty() ||
      !cv::imencode(".jpg", frame.mat, jpeg_buffer, jpeg_params)) {
    // No image: leave image_data empty rather than resending stale bytes.
    reply.set_encoding(sensors::CameraEncoding::UNKNOWN);
    return;
  }

  reply.set_encoding(sensors::CameraEncoding::MJPEG);
  reply.set_image_data(jpeg_buffer.data(), jpeg_buffer.size());
}
