#include "msensor/camera/sim_camera.hh"
#include "msensor/timing/timing.hh"
#include <chrono>

namespace msensor {

namespace {
constexpr auto kPeriod = std::chrono::milliseconds(33); // ~30 FPS
}

void SimCamera::startSampling() {
  sampler_.startPeriodic(kPeriod, [this] {
    // No consumer: produce nothing, so sequence numbers stay unconsumed.
    if (callback_.hasCallback()) {
      CameraFrame frame;
      frame.mat = image_.clone();
      frame.header = {timing::getNowNs(), sequence_number_++};
      callback_.emit(std::move(frame));
    }
  });
}

void SimCamera::stopSampling() {
  sampler_.stop();
}

void SimCamera::setFrameCallback(FrameCallback callback) {
  callback_.setCallback(std::move(callback));
}

cv::Mat SimCamera::makeImage() {
  // A simple synthetic image (a gradient).
  constexpr int width = 640;
  constexpr int height = 480;
  constexpr int factor = 256;

  cv::Mat image(height, width, CV_8UC3);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      image.at<cv::Vec3b>(y, x) =
          cv::Vec3b(x % factor, y % factor, (x + y) % factor);
    }
  }
  return image;
}

bool SimCamera::isOpened() const { return true; }

void SimCamera::release() {}

} // namespace msensor
