#pragma once

#include <opencv2/core.hpp>

#include "interface/Header.hh"

namespace msensor {

struct CameraFrame {
  Header header;
  cv::Mat mat;
};

class ICamera {
public:
  virtual ~ICamera() = default;

  virtual bool read(CameraFrame &frame) = 0;
  virtual bool isOpened() const = 0;
  virtual void release() = 0;
};

} // namespace msensor
