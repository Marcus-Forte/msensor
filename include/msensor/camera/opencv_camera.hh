#pragma once

#include <string>

#include "msensor/interface/CallbackSlot.hh"
#include "msensor/interface/SamplingThread.hh"
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
   * @param encoding What the capture delivers (see CameraFrameEncoding).
   */
  explicit OpenCvCamera(
      int device_id, CameraFrameEncoding encoding = CameraFrameEncoding::Raw);

  /**
   * @brief Construct a new OpenCV Camera object.
   *
   * @param pipeline GStreamer pipeline string for advanced camera
   * configurations.
   * @param encoding What the pipeline delivers. Use Jpeg for pipelines that
   * end in already-encoded JPEG (e.g. a camera's native MJPEG, or
   * `... ! jpegenc ! appsink`): OpenCV's GStreamer backend then returns the
   * encoded buffer as a 1xN CV_8UC1 matrix.
   */
  explicit OpenCvCamera(
      std::string &&pipeline,
      CameraFrameEncoding encoding = CameraFrameEncoding::Raw);

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

  const CameraFrameEncoding encoding_;
  cv::VideoCapture m_capture; ///< OpenCV video capture object.
  uint32_t sequence_number_ = 0; ///< Capture thread only.
  CallbackSlot<CameraFrame> callback_;
  SamplingThread sampler_; // last: joined before the members above are destroyed
};

} // namespace msensor
