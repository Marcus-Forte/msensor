#pragma once

#include <stop_token>
#include <string>
#include <thread>

#include "msensor/interface/CallbackSlot.hh"
#include "msensor/interface/ICamera.hh"
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
   * Automatically stops capture and releases camera resources.
   */
  ~OpenCvCamera() override;

  /// Start the capture loop. No-op if already running.
  void startSampling() override;
  /// Stop the capture loop. No-op if not running.
  void stopSampling() override;
  /// Register the single frame consumer.
  void setFrameCallback(FrameCallback callback) override;

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
  /// Capture a single frame. Returns false on error or if not opened.
  bool capture(CameraFrame &frame);
  /// Capture loop; runs on the capture thread.
  void run(std::stop_token st);

  cv::VideoCapture m_capture; ///< OpenCV video capture object.
  CallbackSlot<CameraFrame> callback_;
  std::jthread thread_; // last: joined before the members above are destroyed
};

} // namespace msensor
