#pragma once

#include <opencv2/core.hpp>

namespace msensor {

class ICamera {
public:
  virtual ~ICamera() = default;

  virtual bool read(cv::Mat &frame) = 0;
  virtual bool isOpened() const = 0;
  virtual void release() = 0;
};

} // namespace msensor
