#pragma once

#include "interface/ICamera.hh"

namespace msensor {

/**
 * @brief Simulated camera implementation.
 *
 */
class SimCamera : public ICamera {
public:
  virtual bool read(CameraFrame &frame) override;
  virtual bool isOpened() const override;
  virtual void release() override;
};

} // namespace msensor