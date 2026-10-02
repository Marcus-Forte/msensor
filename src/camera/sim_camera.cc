#include "msensor/camera/sim_camera.hh"
#include "msensor/timing/timing.hh"
#include <chrono>

namespace msensor {

namespace {
constexpr auto kPeriod = std::chrono::milliseconds(33); // ~30 FPS
}

void SimCamera::startSampling() {
  if (thread_.joinable()) {
    return;
  }
  thread_ = std::jthread([this](std::stop_token st) { run(st); });
}

void SimCamera::stopSampling() {
  if (!thread_.joinable()) {
    return;
  }
  thread_.request_stop();
  thread_.join();
}

void SimCamera::setFrameCallback(FrameCallback callback) {
  callback_.setCallback(std::move(callback));
}

void SimCamera::run(std::stop_token st) {
  while (!st.stop_requested()) {
    callback_.emit(capture());
    std::this_thread::sleep_for(kPeriod);
  }
}

CameraFrame SimCamera::capture() {
  // Generate a simple synthetic image (e.g., a gradient)
  constexpr int width = 640;
  constexpr int height = 480;
  constexpr int factor = 256;

  CameraFrame frame;
  frame.mat.create(height, width, CV_8UC3);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      frame.mat.at<cv::Vec3b>(y, x) =
          cv::Vec3b(x % factor, y % factor, (x + y) % factor);
    }
  }
  static uint32_t sequence_number = 0;
  frame.header = {timing::getNowNs(), sequence_number++};
  return frame;
}

bool SimCamera::isOpened() const { return true; }

void SimCamera::release() {}

} // namespace msensor
