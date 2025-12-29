#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <thread>

#include "camera/opencv_camera.hh"
#include "sensors_server.hh"
#include "timing/timing.hh"

using namespace msensor;

static void printUsage() {
  std::cout << "Usage: camera_publisher [camera_index]" << std::endl;
}
int main(int argc, char **argv) {

  if (argc != 2) {
    printUsage();
    return -1;
  }

  OpenCvCamera camera(std::stoi(argv[1]));

  if (!camera.isOpened()) {
    std::cerr << "Error: Could not open camera." << std::endl;
    return -1;
  }

  SensorsServer server;
  server.start();

  cv::Mat frame;

  while (true) {
    const auto now = timing::getNowUs();

    if (camera.read(frame)) {

      server.publishCameraFrame(frame);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}