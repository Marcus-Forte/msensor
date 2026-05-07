#include "config/config.hh"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <json/json.h>

namespace msensor {
namespace {

bool readBoolMember(const Json::Value &object, const char *key,
                    bool default_value) {
  if (!object.isObject() || !object.isMember(key)) {
    return default_value;
  }

  const auto &value = object[key];
  if (!value.isBool()) {
    throw std::runtime_error(std::string("Expected boolean for '") + key +
                             "'.");
  }

  return value.asBool();
}

const Json::Value *readObjectMember(const Json::Value &object,
                                    const char *key) {
  if (!object.isObject() || !object.isMember(key)) {
    return nullptr;
  }

  const auto &value = object[key];
  if (!value.isObject()) {
    throw std::runtime_error(std::string("Expected object for '") + key +
                             "'.");
  }

  return &value;
}

std::string readStringMember(const Json::Value &object, const char *key,
                             const std::string &default_value) {
  if (!object.isObject() || !object.isMember(key)) {
    return default_value;
  }

  const auto &value = object[key];
  if (!value.isString()) {
    throw std::runtime_error(std::string("Expected string for '") + key +
                             "'.");
  }

  return value.asString();
}

int readIntMember(const Json::Value &object, const char *key,
                  int default_value) {
  if (!object.isObject() || !object.isMember(key)) {
    return default_value;
  }

  const auto &value = object[key];
  if (!value.isInt()) {
    throw std::runtime_error(std::string("Expected integer for '") + key +
                             "'.");
  }

  return value.asInt();
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
      readBoolMember(*use_sensors, "lidar", config.rplidar.enable);
    config.camera.enable =
      readBoolMember(*use_sensors, "camera", config.camera.enable);
    config.icm20948.enable =
      readBoolMember(*use_sensors, "icm20948", config.icm20948.enable);
    config.ads1115.enable =
      readBoolMember(*use_sensors, "ads1115", config.ads1115.enable);
  }

    config.ads1115.i2c_bus =
      readIntMember(document, "i2c_bus", config.ads1115.i2c_bus);
    config.rplidar.device =
      readStringMember(document, "lidar_device", config.rplidar.device.string());
    config.camera.pipeline =
      readStringMember(document, "camera_pipeline", config.camera.pipeline);

  if (const auto *icm20948 = readObjectMember(document, "icm20948")) {
    config.icm20948.enable =
      readBoolMember(*icm20948, "enable", config.icm20948.enable);
  }

  if (const auto *ads1115 = readObjectMember(document, "ads1115")) {
    config.ads1115.enable =
      readBoolMember(*ads1115, "enable", config.ads1115.enable);
    config.ads1115.i2c_bus =
      readIntMember(*ads1115, "i2c_bus", config.ads1115.i2c_bus);
  }

  if (const auto *rplidar = readObjectMember(document, "rplidar")) {
    config.rplidar.enable =
      readBoolMember(*rplidar, "enable", config.rplidar.enable);
    config.rplidar.device =
      readStringMember(*rplidar, "device", config.rplidar.device.string());
  }

  if (const auto *camera = readObjectMember(document, "camera")) {
    config.camera.enable = readBoolMember(*camera, "enable", config.camera.enable);
    config.camera.pipeline =
      readStringMember(*camera, "pipeline", config.camera.pipeline);
  }

  if (const auto *mid360 = readObjectMember(document, "mid360")) {
    config.mid360.enable = readBoolMember(*mid360, "enable", config.mid360.enable);
    config.mid360.config = readStringMember(*mid360, "config", "");
  }

  return config;
}

std::filesystem::path Config::defaultConfigPath() {
  return "/cfg/publisher_config.json";
}

} // namespace msensor