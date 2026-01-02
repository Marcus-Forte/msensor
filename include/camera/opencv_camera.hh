#pragma once

#include "interface/ICamera.hh"
#include <opencv2/videoio.hpp>

namespace msensor {

/**
 * @brief OpenCV-based camera implementation using cv::VideoCapture.
 *
 * Wraps OpenCV's VideoCapture API to provide a consistent camera interface.
 * Supports standard USB cameras, built-in webcams, and other video sources
 * accessible through OpenCV's video I/O module.
 */
class OpenCvCamera : public ICamera {
public:
  /**
   * @brief Construct a new OpenCV Camera object.
   *
   * @param device_id Camera device index (0 for default camera, 1 for second,
   * etc.) or video file path. Passed directly to cv::VideoCapture::open().
   */
  OpenCvCamera(int device_id);

  /**
   * @brief Construct a new OpenCV Camera object.
   *
   * @param pipeline GStreamer pipeline string for advanced camera
   * configurations.
   */
  OpenCvCamera(std::string &&pipeline);

  /**
   * @brief Destroy the OpenCV Camera object.
   *
   * Automatically releases camera resources.
   */
  ~OpenCvCamera() override;

  /**
   * @brief Capture a single frame from the camera.
   *
   * @param frame Output parameter where the captured frame will be stored.
   *              The Mat is reused to avoid reallocations in video loops.
   * @return true if frame was successfully captured, false on error or if
   * camera is not opened.
   */
  bool read(cv::Mat &frame) override;

  /**
   * @brief Check if the camera is successfully opened and ready to capture.
   *
   * @return true if camera is opened, false otherwise.
   */
  bool isOpened() const override;

  /**
   * @brief Release camera resources and close the device.
   *
   * Safe to call multiple times. After calling, isOpened() will return false.
   */
  void release() override;

private:
  cv::VideoCapture m_capture; ///< OpenCV video capture object.
};

} // namespace msensor