#include "msensor/camera/opencv_camera.hh"
#include "msensor/timing/timing.hh"

#include <opencv2/videoio.hpp>

#include <chrono>
#include <thread>

namespace msensor {

OpenCvCamera::OpenCvCamera(std::string &&pipeline) {
  m_capture.open(pipeline, cv::CAP_GSTREAMER);
}

OpenCvCamera::OpenCvCamera(int device_id) { m_capture.open(device_id); }

OpenCvCamera::~OpenCvCamera() {
  stopSampling();
  release();
}

void OpenCvCamera::startSampling() {
  if (thread_.joinable()) {
    return;
  }
  thread_ = std::jthread([this](std::stop_token st) { run(st); });
}

void OpenCvCamera::stopSampling() {
  if (!thread_.joinable()) {
    return;
  }
  thread_.request_stop();
  thread_.join();
}

void OpenCvCamera::setFrameCallback(FrameCallback callback) {
  callback_.setCallback(std::move(callback));
}

void OpenCvCamera::run(std::stop_token st) {
  while (!st.stop_requested()) {
    CameraFrame frame;
    if (capture(frame)) {
      callback_.emit(std::move(frame));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

bool OpenCvCamera::capture(CameraFrame &frame) {
  if (!isOpened()) {
    return false;
  }
  bool success = m_capture.read(frame.mat);
  if (success) {
    static uint32_t sequence_number = 0;
    frame.header = {timing::getNowNs(), sequence_number++};
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
