#include "camera/opencv_camera.hh"

#include <opencv2/videoio.hpp>

namespace msensor {

OpenCvCamera::OpenCvCamera(std::string pipeline) { m_capture.open(pipeline, cv::CAP_GSTREAMER); }

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