#pragma once

#include <filesystem>
#include <string>

namespace msensor {

class Config {
public:
  struct Icm20948Config {
    bool enable = false;
  } icm20948;

  struct RplidarConfig {
    bool enable = true;
    std::filesystem::path device = "/dev/ttyUSB0";
  } rplidar;

  struct CameraConfig {
    bool enable = true;
    std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,format=BGR,width=1536,height=864,framerate=10/1 ! "
        "videoconvert ! appsink max-buffers=1 drop=true sync=false";
  } camera;

  struct Mid360Config {
    bool enable = false;
    std::string config;
  } mid360;

  struct Ads1115Config {
    bool enable = false;
    int i2c_bus = 1;
  } ads1115;

  static Config fromFile(const std::filesystem::path &config_path);
  static std::filesystem::path defaultConfigPath();
};

} // namespace msensor