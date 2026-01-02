#pragma once

#include <opencv2/core.hpp>

namespace msensor {

struct CameraFrame {
  cv::Mat mat;
  uint64_t timestamp;
};

class ICamera {
public:
  virtual ~ICamera() = default;

  virtual bool read(CameraFrame &frame) = 0;
  virtual bool isOpened() const = 0;
  virtual void release() = 0;
};

} // namespace msensor
