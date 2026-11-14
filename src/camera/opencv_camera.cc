#include "msensor/camera/opencv_camera.hh"
#include "msensor/timing/timing.hh"

#include <opencv2/videoio.hpp>

#include <chrono>
#include <thread>

namespace msensor {

namespace {
constexpr auto kFailBackoff = std::chrono::milliseconds(100);
}

OpenCvCamera::OpenCvCamera(std::string &&pipeline, CameraFrameEncoding encoding)
    : encoding_(encoding) {
  m_capture.open(pipeline, cv::CAP_GSTREAMER);
}

OpenCvCamera::OpenCvCamera(int device_id, CameraFrameEncoding encoding)
    : encoding_(encoding) {
  m_capture.open(device_id);
}

OpenCvCamera::~OpenCvCamera() {
  stopSampling();
  release();
}

void OpenCvCamera::startSampling() {
  sampler_.startFreeRunning([this]() -> std::chrono::nanoseconds {
    if (!callback_.hasCallback()) {
      // No consumer: keep the pipeline draining, but do not decode, copy or
      // allocate, and do not consume a sequence number.
      const bool ok = isOpened() && m_capture.grab();
      return ok ? std::chrono::nanoseconds::zero() : kFailBackoff;
    }
    CameraFrame frame;
    if (capture(frame)) {
      callback_.emit(std::move(frame));
      return std::chrono::nanoseconds::zero();
    }
    return kFailBackoff;
  });
}

void OpenCvCamera::stopSampling() {
  sampler_.stop();
}

void OpenCvCamera::setFrameCallback(FrameCallback callback) {
  callback_.setCallback(std::move(callback));
}

bool OpenCvCamera::capture(CameraFrame &frame) {
  if (!isOpened()) {
    return false;
  }
  bool success = m_capture.read(frame.mat);
  if (success) {
    frame.encoding = encoding_;
    frame.header = {timing::getNowNs(), sequence_number_++};
  }
  return success;
}

bool OpenCvCamera::isOpened() const { return m_capture.isOpened(); }

void OpenCvCamera::release() {
  if (m_capture.isOpened()) {
    m_capture.release();
  }
}

} // namespace msensor
