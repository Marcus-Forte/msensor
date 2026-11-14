#include "msensor/config/config.hh"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#ifndef MSENSOR_SOURCE_DIR
#error "MSENSOR_SOURCE_DIR must be defined"
#endif

namespace {

namespace fs = std::filesystem;
using msensor::Config;

constexpr float DefaultExternalGain = (10.0f + 5.1f) / 5.1f;

class ConfigTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto *info = ::testing::UnitTest::GetInstance()->current_test_info();
    dir_ = fs::temp_directory_path() /
           (std::string("msensor_test_config_") + info->name());
    fs::remove_all(dir_);
    fs::create_directories(dir_);
  }
  void TearDown() override { fs::remove_all(dir_); }

  fs::path write(const std::string &text, const std::string &name = "c.json") {
    const fs::path path = dir_ / name;
    std::ofstream(path) << text;
    return path;
  }
  Config load(const std::string &text) { return Config::fromFile(write(text)); }

  fs::path dir_;
};

TEST_F(ConfigTest, EmptyObjectGivesDefaults) {
  const Config c = load("{}");
  EXPECT_FALSE(c.icm20948.enable);
  EXPECT_EQ(c.icm20948.i2c_bus, 1);
  EXPECT_TRUE(c.rplidar.enable);
  EXPECT_EQ(c.rplidar.device, fs::path("/dev/ttyUSB0"));
  EXPECT_TRUE(c.camera.enable);
  EXPECT_EQ(c.camera.encoding, "raw");
  EXPECT_EQ(c.camera.pipeline, Config{}.camera.pipeline);
  EXPECT_EQ(c.camera.jpeg_quality, 85);
  EXPECT_FALSE(c.mid360.enable);
  EXPECT_TRUE(c.mid360.config.empty());
  EXPECT_EQ(c.mid360.accumulate_scan_count, 100u);
  EXPECT_EQ(c.mid360.mode, "normal");
  EXPECT_EQ(c.mid360.scan_pattern, "non_repetitive");
  EXPECT_FALSE(c.ads1115.enable);
  EXPECT_EQ(c.ads1115.i2c_bus, 1);
  EXPECT_DOUBLE_EQ(c.ads1115.gain, 6.144);
  EXPECT_EQ(c.ads1115.data_rate, 8);
  EXPECT_EQ(c.ads1115.channel, 0);
  EXPECT_FLOAT_EQ(c.ads1115.external_gain, DefaultExternalGain);
  EXPECT_FALSE(c.sim.lidar);
  EXPECT_FALSE(c.sim.imu);
  EXPECT_FALSE(c.sim.camera);
  EXPECT_FALSE(c.sim.adc);
}

TEST_F(ConfigTest, PerSensorKeys) {
  const Config c = load(R"({
    "icm20948": {"enable": true},
    "rplidar": {"enable": false, "device": "/dev/ttyUSB7"},
    "camera": {"enable": false, "pipeline": "videotestsrc ! appsink",
               "encoding": "jpeg"},
    "mid360": {"enable": true, "config": "/abs/m.json",
               "accumulate_scan_count": 7},
    "ads1115": {"enable": true, "i2c_bus": 3}
  })");
  EXPECT_TRUE(c.icm20948.enable);
  EXPECT_FALSE(c.rplidar.enable);
  EXPECT_EQ(c.rplidar.device, fs::path("/dev/ttyUSB7"));
  EXPECT_FALSE(c.camera.enable);
  EXPECT_EQ(c.camera.pipeline, "videotestsrc ! appsink");
  EXPECT_EQ(c.camera.encoding, "jpeg");
  EXPECT_TRUE(c.mid360.enable);
  EXPECT_EQ(c.mid360.config, "/abs/m.json");
  EXPECT_EQ(c.mid360.accumulate_scan_count, 7u);
  EXPECT_TRUE(c.ads1115.enable);
  EXPECT_EQ(c.ads1115.i2c_bus, 3);
}

TEST_F(ConfigTest, LegacyKeys) {
  const Config c = load(R"({
    "use_sensors": {"lidar": false, "camera": false, "icm20948": true,
                    "ads1115": true},
    "i2c_bus": 4,
    "lidar_device": "/dev/ttyLegacy",
    "camera_pipeline": "legacy ! pipeline"
  })");
  EXPECT_FALSE(c.rplidar.enable);
  EXPECT_FALSE(c.camera.enable);
  EXPECT_TRUE(c.icm20948.enable);
  EXPECT_TRUE(c.ads1115.enable);
  EXPECT_EQ(c.ads1115.i2c_bus, 4);
  EXPECT_EQ(c.rplidar.device, fs::path("/dev/ttyLegacy"));
  EXPECT_EQ(c.camera.pipeline, "legacy ! pipeline");
}

TEST_F(ConfigTest, NewKeysHappyPath) {
  const Config c = load(R"({
    "ads1115": {"gain": 2.048, "data_rate": 860, "channel": 3,
                "external_gain": 2.5},
    "camera": {"jpeg_quality": 40},
    "mid360": {"mode": "power_save", "scan_pattern": "low_frame_rate"},
    "sim": {"lidar": true, "imu": true, "camera": true, "adc": true}
  })");
  EXPECT_DOUBLE_EQ(c.ads1115.gain, 2.048);
  EXPECT_EQ(c.ads1115.data_rate, 860);
  EXPECT_EQ(c.ads1115.channel, 3);
  EXPECT_FLOAT_EQ(c.ads1115.external_gain, 2.5f);
  EXPECT_EQ(c.camera.jpeg_quality, 40);
  EXPECT_EQ(c.mid360.mode, "power_save");
  EXPECT_EQ(c.mid360.scan_pattern, "low_frame_rate");
  EXPECT_TRUE(c.sim.lidar);
  EXPECT_TRUE(c.sim.imu);
  EXPECT_TRUE(c.sim.camera);
  EXPECT_TRUE(c.sim.adc);
}

TEST_F(ConfigTest, AllAllowedAds1115Values) {
  for (const char *gain : {"6.144", "4.096", "2.048", "1.024", "0.512", "0.256"}) {
    EXPECT_NO_THROW(load(std::string(R"({"ads1115":{"gain":)") + gain + "}}"))
        << gain;
  }
  for (int rate : {8, 16, 32, 64, 128, 250, 475, 860}) {
    EXPECT_EQ(load("{\"ads1115\":{\"data_rate\":" + std::to_string(rate) + "}}")
                  .ads1115.data_rate,
              rate);
  }
  for (int channel = 0; channel <= 3; ++channel) {
    EXPECT_EQ(
        load("{\"ads1115\":{\"channel\":" + std::to_string(channel) + "}}")
            .ads1115.channel,
        channel);
  }
  for (const char *pattern : {"non_repetitive", "repetitive", "low_frame_rate"}) {
    EXPECT_EQ(load(std::string(R"({"mid360":{"scan_pattern":")") + pattern +
                   "\"}}")
                  .mid360.scan_pattern,
              pattern);
  }
  EXPECT_EQ(load(R"({"camera":{"jpeg_quality":0}})").camera.jpeg_quality, 0);
  EXPECT_EQ(load(R"({"camera":{"jpeg_quality":100}})").camera.jpeg_quality,
            100);
}

TEST_F(ConfigTest, Icm20948BusFallsBackToAds1115Bus) {
  EXPECT_EQ(load("{}").icm20948.i2c_bus, 1);
  EXPECT_EQ(load(R"({"i2c_bus": 5})").icm20948.i2c_bus, 5);
  EXPECT_EQ(load(R"({"ads1115": {"i2c_bus": 6}})").icm20948.i2c_bus, 6);
  // The ADS1115 section wins over the legacy key, and the ICM follows it.
  EXPECT_EQ(load(R"({"i2c_bus": 5, "ads1115": {"i2c_bus": 6}})")
                .icm20948.i2c_bus,
            6);
}

TEST_F(ConfigTest, Icm20948ExplicitBusWins) {
  const Config c =
      load(R"({"i2c_bus": 5, "ads1115": {"i2c_bus": 6},
               "icm20948": {"i2c_bus": 2}})");
  EXPECT_EQ(c.icm20948.i2c_bus, 2);
  EXPECT_EQ(c.ads1115.i2c_bus, 6);
}

TEST_F(ConfigTest, RejectsWrongTypes) {
  for (const char *json : {
           R"({"icm20948": {"enable": "yes"}})",
           R"({"icm20948": {"i2c_bus": "1"}})",
           R"({"icm20948": {"i2c_bus": 1.5}})",
           R"({"icm20948": 3})",
           R"({"rplidar": {"device": 1}})",
           R"({"camera": {"encoding": 1}})",
           R"({"camera": {"jpeg_quality": "85"}})",
           R"({"camera": {"jpeg_quality": 85.5}})",
           R"({"mid360": {"accumulate_scan_count": "1"}})",
           R"({"mid360": {"mode": 1}})",
           R"({"mid360": {"scan_pattern": true}})",
           R"({"mid360": {"config": 5}})",
           R"({"ads1115": {"i2c_bus": "1"}})",
           R"({"ads1115": {"gain": "6.144"}})",
           R"({"ads1115": {"data_rate": "8"}})",
           R"({"ads1115": {"data_rate": 8.5}})",
           R"({"ads1115": {"channel": "0"}})",
           R"({"ads1115": {"external_gain": "2"}})",
           R"({"sim": {"lidar": 1}})",
           R"({"sim": {"imu": "true"}})",
           R"({"sim": {"camera": null}})",
           R"({"sim": {"adc": 0}})",
           R"({"sim": []})",
           R"({"use_sensors": {"lidar": "no"}})",
           R"({"i2c_bus": "1"})",
           R"({"lidar_device": 4})",
           R"({"camera_pipeline": 4})",
       }) {
    EXPECT_THROW(load(json), std::runtime_error) << json;
  }
}

TEST_F(ConfigTest, RejectsOutOfRangeAndUnknownValues) {
  for (const char *json : {
           R"({"ads1115": {"gain": 5.0}})",
           R"({"ads1115": {"gain": 0}})",
           R"({"ads1115": {"gain": -6.144}})",
           R"({"ads1115": {"data_rate": 100}})",
           R"({"ads1115": {"data_rate": 0}})",
           R"({"ads1115": {"channel": 4}})",
           R"({"ads1115": {"channel": -1}})",
           R"({"ads1115": {"external_gain": 0}})",
           R"({"ads1115": {"external_gain": -1.5}})",
           R"({"camera": {"jpeg_quality": 101}})",
           R"({"camera": {"jpeg_quality": -1}})",
           R"({"camera": {"encoding": "png"}})",
           R"({"mid360": {"mode": "turbo"}})",
           R"({"mid360": {"mode": "Normal"}})",
           R"({"mid360": {"scan_pattern": "spiral"}})",
           R"({"mid360": {"accumulate_scan_count": 0}})",
       }) {
    EXPECT_THROW(load(json), std::runtime_error) << json;
  }
}

TEST_F(ConfigTest, ErrorMessagesNameTheKey) {
  try {
    load(R"({"ads1115": {"gain": 5.0}})");
    FAIL();
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("ads1115.gain"), std::string::npos)
        << e.what();
  }
  try {
    load(R"({"camera": {"jpeg_quality": "x"}})");
    FAIL();
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("camera.jpeg_quality"),
              std::string::npos)
        << e.what();
  }
}

TEST_F(ConfigTest, RelativeMid360ConfigResolvesAgainstConfigDirectory) {
  fs::create_directories(dir_ / "sub");
  const fs::path path = dir_ / "sub" / "c.json";
  std::ofstream(path) << R"({"mid360": {"config": "../m/mid.json"}})";
  const Config c = Config::fromFile(path);
  EXPECT_EQ(fs::path(c.mid360.config), (dir_ / "m" / "mid.json"));
}

TEST_F(ConfigTest, AbsoluteMid360ConfigIsKept) {
  const Config c = load(R"({"mid360": {"config": "/etc/mid.json"}})");
  EXPECT_EQ(c.mid360.config, "/etc/mid.json");
}

TEST_F(ConfigTest, EmptyMid360ConfigStaysEmpty) {
  EXPECT_TRUE(load(R"({"mid360": {"enable": true}})").mid360.config.empty());
}

TEST_F(ConfigTest, BadFilesThrow) {
  EXPECT_THROW(load("{not json"), std::runtime_error);
  EXPECT_THROW(load(""), std::runtime_error);
  EXPECT_THROW(load("[]"), std::runtime_error);
  EXPECT_THROW(load("3"), std::runtime_error);
  EXPECT_THROW(Config::fromFile(dir_ / "missing.json"), std::runtime_error);
}

TEST(ConfigShipped, TemplatesParse) {
  const fs::path root = MSENSOR_SOURCE_DIR;
  for (const char *name :
       {"config/publisher_config.json", "deployment/config/publisher_config.json",
        "config/sim_config.json"}) {
    EXPECT_NO_THROW(Config::fromFile(root / name)) << name;
  }
}

TEST(ConfigShipped, SimConfigSelectsOnlySimulators) {
  const Config c =
      Config::fromFile(fs::path(MSENSOR_SOURCE_DIR) / "config/sim_config.json");
  EXPECT_TRUE(c.sim.lidar && c.sim.imu && c.sim.camera && c.sim.adc);
  EXPECT_FALSE(c.rplidar.enable);
  EXPECT_FALSE(c.camera.enable);
  EXPECT_FALSE(c.mid360.enable);
  EXPECT_FALSE(c.icm20948.enable);
  EXPECT_FALSE(c.ads1115.enable);
}

} // namespace
