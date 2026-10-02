#pragma once

#include <functional>
#include <opencv2/core.hpp>

#include "msensor/interface/Header.hh"

namespace msensor {

struct CameraFrame {
  Header header;
  cv::Mat mat;
};

class ICamera {
public:
  /// Callback invoked for every captured frame.
  using FrameCallback = std::function<void(CameraFrame)>;

  virtual ~ICamera() = default;

  /// Start the capture loop. No-op if already running.
  virtual void startSampling() = 0;
  /// Stop the capture loop. No-op if not running.
  virtual void stopSampling() = 0;
  /// Register the single consumer of camera frames. Passing an empty callback
  /// clears it. The callback runs on the capture thread and receives ownership
  /// of each frame.
  virtual void setFrameCallback(FrameCallback callback) = 0;

  virtual bool isOpened() const = 0;
  virtual void release() = 0;
};

} // namespace msensor
