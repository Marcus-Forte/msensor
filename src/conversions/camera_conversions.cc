#include <opencv2/imgcodecs.hpp>

#include "msensor/conversions/camera_conversions.hh"

#include <vector>

namespace msensor {

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

void toProtobuf(const CameraFrame &frame,
                sensors::CameraStreamReply &reply, int quality) {
  reply.Clear();
  reply.mutable_header()->set_timestamp(frame.header.timestamp);
  reply.mutable_header()->set_sequence_number(frame.header.sequence_number);

  if (frame.encoding == CameraFrameEncoding::Jpeg) {
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

} // namespace msensor
