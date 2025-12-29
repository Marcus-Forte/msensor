#include "camera/opencv_camera.hh"

#include <opencv2/videoio.hpp>

namespace msensor {

OpenCvCamera::OpenCvCamera(int device_id) { m_capture.open(device_id); }

OpenCvCamera::~OpenCvCamera() { release(); }

bool OpenCvCamera::read(cv::Mat &frame) {
  if (!isOpened()) {
    return false;
  }
  return m_capture.read(frame);
}

bool OpenCvCamera::isOpened() const { return m_capture.isOpened(); }

void OpenCvCamera::release() {
  if (m_capture.isOpened()) {
    m_capture.release();
  }
}

} // namespace msensor