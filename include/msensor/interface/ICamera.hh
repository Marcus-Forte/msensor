#pragma once

#include <functional>
#include <opencv2/core.hpp>

#include "msensor/interface/Header.hh"

namespace msensor {

/// How `CameraFrame::mat` is to be interpreted.
enum class CameraFrameEncoding {
  /// Decoded pixels (BGR or gray image).
  Raw,
  /// An already JPEG-encoded image: `mat` holds the encoded bytes as a
  /// single-row or single-column CV_8UC1 matrix.
  Jpeg,
};

struct CameraFrame {
  Header header;
  cv::Mat mat;
  CameraFrameEncoding encoding = CameraFrameEncoding::Raw;
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
  /// of each frame. When this returns, no invocation of the previous callback
  /// is still running (implementations use CallbackSlot).
  virtual void setFrameCallback(FrameCallback callback) = 0;

  virtual bool isOpened() const = 0;
  virtual void release() = 0;
};

} // namespace msensor
