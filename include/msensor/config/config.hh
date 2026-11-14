#pragma once

#include <filesystem>
#include <cstddef>
#include <string>

namespace msensor {

class Config {
public:
  struct Icm20948Config {
    bool enable = false;
    /// I2C bus index. When the file does not set it, fromFile() uses the
    /// ADS1115 section's bus (the historical behaviour).
    int i2c_bus = 1;
  } icm20948;

  struct RplidarConfig {
    bool enable = true;
    std::filesystem::path device = "/dev/ttyUSB0";
  } rplidar;

  struct CameraConfig {
    bool enable = true;
    /// What the pipeline delivers: "raw" (decoded pixels) or "jpeg"
    /// (already-encoded JPEG frames, passed to clients without re-encoding).
    std::string encoding = "raw";
    std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,format=BGR,width=1536,height=864,framerate=10/1 ! "
        "videoconvert ! appsink max-buffers=1 drop=true sync=false";
    int jpeg_quality = 85; ///< 0..100, used when raw frames are encoded
  } camera;

  struct Mid360Config {
    bool enable = false;
    std::string config;
    std::size_t accumulate_scan_count = 100;
    /// "normal" or "power_save".
    std::string mode = "normal";
    /// "non_repetitive", "repetitive" or "low_frame_rate"; ignored in
    /// "power_save" mode.
    std::string scan_pattern = "non_repetitive";
  } mid360;

  struct Ads1115Config {
    bool enable = false;
    int i2c_bus = 1;
    /// Full-scale range in volts: 6.144, 4.096, 2.048, 1.024, 0.512 or 0.256.
    double gain = 6.144;
    /// Samples per second: 8, 16, 32, 64, 128, 250, 475 or 860.
    int data_rate = 8;
    /// Single-ended input channel, 0..3.
    int channel = 0;
    /// Voltage divider factor in front of the input; positive.
    float external_gain = (10.0f + 5.1f) / 5.1f; // 10k and 5.1k resistors
  } ads1115;

  /// Select the simulated drivers; each takes precedence over the hardware
  /// driver of the same kind.
  struct SimConfig {
    bool lidar = false;
    bool imu = false;
    bool camera = false;
    bool adc = false;
  } sim;

  /// Throws std::runtime_error if the file cannot be read or parsed, or a
  /// value has the wrong type or is not allowed. A relative mid360.config is
  /// resolved against the directory of config_path.
  static Config fromFile(const std::filesystem::path &config_path);
  static std::filesystem::path defaultConfigPath();
};

} // namespace msensor