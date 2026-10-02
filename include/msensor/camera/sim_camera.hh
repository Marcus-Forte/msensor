#pragma once

#include <stop_token>
#include <thread>

#include "msensor/interface/CallbackSlot.hh"
#include "msensor/interface/ICamera.hh"

namespace msensor {

/**
 * @brief Simulated camera implementation.
 *
 */
class SimCamera : public ICamera {
public:
  /// Start the synthetic capture loop (~30 FPS).
  void startSampling() override;
  /// Stop the capture loop.
  void stopSampling() override;
  /// Register the single frame consumer.
  void setFrameCallback(FrameCallback callback) override;

  virtual bool isOpened() const override;
  virtual void release() override;

private:
  CameraFrame capture();
  void run(std::stop_token st);

  CallbackSlot<CameraFrame> callback_;
  std::jthread thread_; // last: joined before the members above are destroyed
};

} // namespace msensor
