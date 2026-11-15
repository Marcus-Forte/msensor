#include "msensor/config/config.hh"

#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <json/json.h>

namespace msensor {
namespace {

std::string qualified(const char *section, const char *key) {
  return *section ? std::string(section) + "." + key : std::string(key);
}

bool readBoolMember(const Json::Value &object, const char *key,
                    bool default_value, const char *section = "") {
  if (!object.isObject() || !object.isMember(key)) {
    return default_value;
  }

  const auto &value = object[key];
  if (!value.isBool()) {
    throw std::runtime_error("Expected boolean for '" +
                             qualified(section, key) + "'.");
  }

  return value.asBool();
}

double readNumberMember(const Json::Value &object, const char *key,
                        double default_value, const char *section = "") {
  if (!object.isObject() || !object.isMember(key)) {
    return default_value;
  }

  const auto &value = object[key];
  if (!value.isNumeric()) {
    throw std::runtime_error("Expected number for '" +
                             qualified(section, key) + "'.");
  }

  return value.asDouble();
}

template <typename T, std::size_t N>
bool oneOf(T value, const T (&allowed)[N]) {
  for (const T &candidate : allowed) {
    if (value == candidate) {
      return true;
    }
  }
  return false;
}

const Json::Value *readObjectMember(const Json::Value &object,
                                    const char *key) {
  if (!object.isObject() || !object.isMember(key)) {
    return nullptr;
  }

  const auto &value = object[key];
  if (!value.isObject()) {
    throw std::runtime_error(std::string("Expected object for '") + key + "'.");
  }

  return &value;
}

std::string readStringMember(const Json::Value &object, const char *key,
                             const std::string &default_value,
                             const char *section = "") {
  if (!object.isObject() || !object.isMember(key)) {
    return default_value;
  }

  const auto &value = object[key];
  if (!value.isString()) {
    throw std::runtime_error("Expected string for '" +
                             qualified(section, key) + "'.");
  }

  return value.asString();
}

int readIntMember(const Json::Value &object, const char *key,
                  int default_value, const char *section = "") {
  if (!object.isObject() || !object.isMember(key)) {
    return default_value;
  }

  const auto &value = object[key];
  if (!value.isInt()) {
    throw std::runtime_error("Expected integer for '" +
                             qualified(section, key) + "'.");
  }

  return value.asInt();
}

/// Reads a string member that must be one of `allowed`.
template <std::size_t N>
std::string readChoiceMember(const Json::Value &object, const char *key,
                             const std::string &default_value,
                             const char *section,
                             const char *const (&allowed)[N]) {
  std::string value = readStringMember(object, key, default_value, section);
  for (const char *candidate : allowed) {
    if (value == candidate) {
      return value;
    }
  }
  std::string message =
      "'" + qualified(section, key) + "' must be one of:";
  for (const char *candidate : allowed) {
    message += std::string(" \"") + candidate + "\"";
  }
  throw std::runtime_error(message + ".");
}

} // namespace

Config Config::fromFile(const std::filesystem::path &config_path) {
  std::ifstream config_stream(config_path);
  if (!config_stream) {
    throw std::runtime_error("Unable to open config file: " +
                             config_path.string());
  }

  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;

  Json::Value document;
  std::string parse_errors;
  if (!Json::parseFromStream(builder, config_stream, &document,
                             &parse_errors)) {
    // jsoncpp reports multi-line text; keep the message on one line.
    for (char &c : parse_errors) {
      if (c == '\n') {
        c = ' ';
      }
    }
    while (!parse_errors.empty() && parse_errors.back() == ' ') {
      parse_errors.pop_back();
    }
    std::ostringstream message;
    message << "Failed to parse config file: " << parse_errors;
    throw std::runtime_error(message.str());
  }

  if (!document.isObject()) {
    throw std::runtime_error("Top-level config must be a JSON object.");
  }

  Config config;

  if (const auto *use_sensors = readObjectMember(document, "use_sensors")) {
    config.rplidar.enable =
        readBoolMember(*use_sensors, "lidar", config.rplidar.enable, "use_sensors");
    config.camera.enable =
        readBoolMember(*use_sensors, "camera", config.camera.enable, "use_sensors");
    config.icm20948.enable =
        readBoolMember(*use_sensors, "icm20948", config.icm20948.enable, "use_sensors");
    config.ads1115.enable =
        readBoolMember(*use_sensors, "ads1115", config.ads1115.enable, "use_sensors");
  }

  config.ads1115.i2c_bus =
      readIntMember(document, "i2c_bus", config.ads1115.i2c_bus);
  config.rplidar.device = readStringMember(document, "lidar_device",
                                           config.rplidar.device.string());
  config.camera.pipeline =
      readStringMember(document, "camera_pipeline", config.camera.pipeline);

  // The ICM20948 shares the ADS1115 bus unless it names its own; the ADS1115
  // value is only final after the sections below are parsed.
  std::optional<int> icm20948_bus;

  if (const auto *icm20948 = readObjectMember(document, "icm20948")) {
    if (icm20948->isMember("i2c_bus")) {
      icm20948_bus = readIntMember(*icm20948, "i2c_bus", 0, "icm20948");
    }
    config.icm20948.enable =
        readBoolMember(*icm20948, "enable", config.icm20948.enable, "icm20948");
  }

  if (const auto *ads1115 = readObjectMember(document, "ads1115")) {
    config.ads1115.enable =
        readBoolMember(*ads1115, "enable", config.ads1115.enable, "ads1115");
    config.ads1115.i2c_bus =
        readIntMember(*ads1115, "i2c_bus", config.ads1115.i2c_bus, "ads1115");

    config.ads1115.gain =
        readNumberMember(*ads1115, "gain", config.ads1115.gain, "ads1115");
    bool gain_ok = false;
    for (const double allowed : {6.144, 4.096, 2.048, 1.024, 0.512, 0.256}) {
      gain_ok = gain_ok || std::fabs(config.ads1115.gain - allowed) < 1e-9;
    }
    if (!gain_ok) {
      throw std::runtime_error("'ads1115.gain' must be one of 6.144, 4.096, "
                               "2.048, 1.024, 0.512, 0.256 (volts).");
    }

    config.ads1115.data_rate = readIntMember(*ads1115, "data_rate",
                                             config.ads1115.data_rate,
                                             "ads1115");
    if (!oneOf(config.ads1115.data_rate,
               {8, 16, 32, 64, 128, 250, 475, 860})) {
      throw std::runtime_error("'ads1115.data_rate' must be one of 8, 16, "
                               "32, 64, 128, 250, 475, 860 (samples per "
                               "second).");
    }

    config.ads1115.channel =
        readIntMember(*ads1115, "channel", config.ads1115.channel, "ads1115");
    if (config.ads1115.channel < 0 || config.ads1115.channel > 3) {
      throw std::runtime_error("'ads1115.channel' must be in [0, 3].");
    }

    const double external_gain = readNumberMember(
        *ads1115, "external_gain", config.ads1115.external_gain, "ads1115");
    if (!(external_gain > 0.0) || !std::isfinite(external_gain)) {
      throw std::runtime_error("'ads1115.external_gain' must be positive.");
    }
    config.ads1115.external_gain = static_cast<float>(external_gain);
  }

  if (const auto *rplidar = readObjectMember(document, "rplidar")) {
    config.rplidar.enable =
        readBoolMember(*rplidar, "enable", config.rplidar.enable, "rplidar");
    config.rplidar.device =
        readStringMember(*rplidar, "device", config.rplidar.device.string(),
                         "rplidar");
  }

  if (const auto *camera = readObjectMember(document, "camera")) {
    config.camera.enable =
        readBoolMember(*camera, "enable", config.camera.enable, "camera");
    config.camera.pipeline =
        readStringMember(*camera, "pipeline", config.camera.pipeline, "camera");
    config.camera.encoding =
        readStringMember(*camera, "encoding", config.camera.encoding, "camera");
    if (config.camera.encoding != "raw" && config.camera.encoding != "jpeg") {
      throw std::runtime_error(
          "'camera.encoding' must be \"raw\" or \"jpeg\".");
    }
    config.camera.jpeg_quality = readIntMember(
        *camera, "jpeg_quality", config.camera.jpeg_quality, "camera");
    if (config.camera.jpeg_quality < 0 || config.camera.jpeg_quality > 100) {
      throw std::runtime_error("'camera.jpeg_quality' must be in [0, 100].");
    }
  }

  if (const auto *mid360 = readObjectMember(document, "mid360")) {
    config.mid360.enable =
        readBoolMember(*mid360, "enable", config.mid360.enable, "mid360");
    config.mid360.config = readStringMember(*mid360, "config", "", "mid360");
    const int accumulate_scan_count = readIntMember(
        *mid360, "accumulate_scan_count",
        static_cast<int>(config.mid360.accumulate_scan_count), "mid360");
    if (accumulate_scan_count <= 0) {
      throw std::runtime_error("'mid360.accumulate_scan_count' must be positive.");
    }
    config.mid360.accumulate_scan_count =
        static_cast<std::size_t>(accumulate_scan_count);
    config.mid360.mode = readChoiceMember(*mid360, "mode", config.mid360.mode,
                                          "mid360", {"normal", "power_save"});
    config.mid360.scan_pattern = readChoiceMember(
        *mid360, "scan_pattern", config.mid360.scan_pattern, "mid360",
        {"non_repetitive", "repetitive", "low_frame_rate"});

    // A relative Mid360 config path is relative to the config file.
    if (!config.mid360.config.empty()) {
      const std::filesystem::path mid360_config_path(config.mid360.config);
      if (mid360_config_path.is_relative()) {
        config.mid360.config =
            (config_path.parent_path() / mid360_config_path)
                .lexically_normal()
                .string();
      }
    }
  }

  if (const auto *sim = readObjectMember(document, "sim")) {
    config.sim.lidar = readBoolMember(*sim, "lidar", config.sim.lidar, "sim");
    config.sim.imu = readBoolMember(*sim, "imu", config.sim.imu, "sim");
    config.sim.camera =
        readBoolMember(*sim, "camera", config.sim.camera, "sim");
    config.sim.adc = readBoolMember(*sim, "adc", config.sim.adc, "sim");
  }

  if (const auto *recording = readObjectMember(document, "recording")) {
    config.recording.enable = readBoolMember(
        *recording, "enable", config.recording.enable, "recording");
    config.recording.file = readStringMember(
        *recording, "file", config.recording.file, "recording");
    const int scan_queue = readIntMember(
        *recording, "scan_queue", static_cast<int>(config.recording.scan_queue),
        "recording");
    if (scan_queue <= 0) {
      throw std::runtime_error("'recording.scan_queue' must be positive.");
    }
    config.recording.scan_queue = static_cast<std::size_t>(scan_queue);
    const int imu_queue = readIntMember(
        *recording, "imu_queue", static_cast<int>(config.recording.imu_queue),
        "recording");
    if (imu_queue <= 0) {
      throw std::runtime_error("'recording.imu_queue' must be positive.");
    }
    config.recording.imu_queue = static_cast<std::size_t>(imu_queue);
  }

  config.icm20948.i2c_bus = icm20948_bus.value_or(config.ads1115.i2c_bus);

  return config;
}

std::filesystem::path Config::defaultConfigPath() {
  return "/usr/local/etc/publisher_config.json";
}

} // namespace msensor