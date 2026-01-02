#include "camera/opencv_camera.hh"
#include "timing/timing.hh"

#include <opencv2/videoio.hpp>

namespace msensor {

OpenCvCamera::OpenCvCamera(std::string &&pipeline) {
  m_capture.open(pipeline, cv::CAP_GSTREAMER);
}

OpenCvCamera::OpenCvCamera(int device_id) { m_capture.open(device_id); }

OpenCvCamera::~OpenCvCamera() { release(); }

bool OpenCvCamera::read(CameraFrame &frame) {
  if (!isOpened()) {
    return false;
  }
  bool success = m_capture.read(frame.mat);
  if (success) {
    frame.timestamp = timing::getNowUs();
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