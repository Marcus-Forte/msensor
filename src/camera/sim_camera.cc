#include "camera/sim_camera.hh"
#include "timing/timing.hh"
#include <thread>

namespace msensor {

bool SimCamera::read(CameraFrame &frame) {
  // Generate a simple synthetic image (e.g., a gradient)
  constexpr int width = 640;
  constexpr int height = 480;
  constexpr int factor = 256;

  frame.mat.create(height, width, CV_8UC3);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      frame.mat.at<cv::Vec3b>(y, x) =
          cv::Vec3b(x % factor, y % factor, (x + y) % factor);
    }
  }
  frame.timestamp = timing::getNowUs();

  std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
  return true;
}

bool SimCamera::isOpened() const { return true; }

void SimCamera::release() {}

} // namespace msensor