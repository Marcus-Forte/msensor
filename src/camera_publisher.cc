#include <format>
#include <iostream>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/imgcodecs.hpp>
#include <thread>

#include "camera/opencv_camera.hh"
#include "sensors_server.hh"

using namespace msensor;

constexpr const char *pipeline_example =
    "libcamerasrc ! "
    "video/x-raw,format=BGR,width=1536,height=864,framerate=30/1 ! "
    "videoconvert ! appsink max-buffers=1 drop=true sync=false";

static void printUsage() {
  std::cout << "Usage: camera_publisher [camera gstream]" << std::endl;
  std::cout << std::format("Example gstreamer pipeline: {}", pipeline_example)
            << std::endl;
}
int main(int argc, char **argv) {

  if (argc != 2) {
    printUsage();
    return -1;
  }

  // add opencv logs
  cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_DEBUG);

  OpenCvCamera camera(argv[1]);

  if (!camera.isOpened()) {
    std::cerr << "Error: Could not open camera." << std::endl;
    return -1;
  }

  SensorsServer server(nullptr, std::make_shared<OpenCvCamera>(camera), nullptr,
                       nullptr);
  server.start();

  while (true) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}