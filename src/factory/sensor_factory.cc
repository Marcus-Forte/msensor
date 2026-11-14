#include "msensor/factory/sensor_factory.hh"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include "msensor/adc/ADS1115.hh"
#include "msensor/adc/sim_adc.hh"
#include "msensor/camera/opencv_camera.hh"
#include "msensor/camera/sim_camera.hh"
#include "msensor/imu/icm-20948.h"
#include "msensor/imu/icm-20948_defs.h"
#include "msensor/imu/sim_imu.hh"
#include "msensor/lidar/mid360.hh"
#include "msensor/lidar/rp_lidar.hh"
#include "msensor/lidar/sim_lidar.hh"
#include "msensor/log/log.hh"

namespace msensor {
namespace {

constexpr uint8_t DefaultADSAddress = 0x48;

ADS1115::Gain toAdsGain(double volts) {
  // Config has already restricted the value to the six full-scale ranges.
  if (volts > 5.0) return ADS1115::Gain::PLUS_MINUS_6_144;
  if (volts > 3.0) return ADS1115::Gain::PLUS_MINUS_4_096;
  if (volts > 1.5) return ADS1115::Gain::PLUS_MINUS_2_048;
  if (volts > 0.75) return ADS1115::Gain::PLUS_MINUS_1_024;
  if (volts > 0.384) return ADS1115::Gain::PLUS_MINUS_0_512;
  return ADS1115::Gain::PLUS_MINUS_0_256;
}

ADS1115::DataRate toAdsDataRate(int sps) {
  switch (sps) {
  case 16: return ADS1115::DataRate::SPS_16;
  case 32: return ADS1115::DataRate::SPS_32;
  case 64: return ADS1115::DataRate::SPS_64;
  case 128: return ADS1115::DataRate::SPS_128;
  case 250: return ADS1115::DataRate::SPS_250;
  case 475: return ADS1115::DataRate::SPS_475;
  case 860: return ADS1115::DataRate::SPS_860;
  default: return ADS1115::DataRate::SPS_8;
  }
}

} // namespace

Sensors makeSensors(const Config &config) {
  Sensors sensors;

  // The Mid360 is both a lidar and an IMU; it is only needed if at least one
  // of its two roles is not taken over by a simulator.
  const bool use_mid360 =
      config.mid360.enable && !(config.sim.lidar && config.sim.imu);

  if (config.mid360.enable && config.rplidar.enable && !config.sim.lidar) {
    log::warn("Both mid360 and rplidar are enabled; preferring mid360.");
  }

  if (use_mid360) {
    log::info("Mid360 accumulation samples: {}",
              config.mid360.accumulate_scan_count);
    if (config.mid360.config.empty()) {
      throw std::runtime_error(
          "mid360 is enabled but no config path was provided.");
    }
    if (!std::filesystem::exists(config.mid360.config)) {
      throw std::runtime_error("mid360 config file: " + config.mid360.config +
                               " does not exist.");
    }
    auto mid360 = std::make_shared<Mid360>(std::string(config.mid360.config),
                                           config.mid360.accumulate_scan_count);
    mid360->init();
    if (config.mid360.mode == "power_save") {
      mid360->setMode(Mid360::Mode::PowerSave);
    } else {
      mid360->setMode(Mid360::Mode::Normal);
      if (config.mid360.scan_pattern == "repetitive") {
        mid360->setScanPattern(Mid360::ScanPattern::Repetitive);
      } else if (config.mid360.scan_pattern == "low_frame_rate") {
        mid360->setScanPattern(Mid360::ScanPattern::LowFrameRate);
      } else {
        mid360->setScanPattern(Mid360::ScanPattern::NonRepetitive);
      }
    }
    mid360->startSampling();
    if (!config.sim.lidar) {
      sensors.lidar = mid360;
    }
    if (!config.sim.imu) {
      sensors.imu = mid360;
      if (config.icm20948.enable) {
        log::warn("mid360 already provides IMU data; skipping standalone "
                  "ICM20948 initialization.");
      }
    }
  } else if (config.rplidar.enable && !config.sim.lidar) {
    if (std::filesystem::exists(config.rplidar.device)) {
      auto rp = std::make_shared<RPLidar>(config.rplidar.device);
      rp->init();
      rp->setMotorRPM(360);
      rp->startSampling();
      sensors.lidar = rp;
    } else {
      log::warn("Lidar device: {} does not exist. Lidar will be unavailable.",
                config.rplidar.device.string());
    }
  }

  if (config.sim.lidar) {
    if (config.mid360.enable || config.rplidar.enable) {
      log::warn("sim.lidar overrides the enabled hardware lidar.");
    }
    auto sim = std::make_shared<SimLidar>(false);
    sim->startSampling();
    sensors.lidar = sim;
  }

  if (config.sim.adc) {
    if (config.ads1115.enable) {
      log::warn("sim.adc overrides the enabled ads1115.");
    }
    sensors.adc = std::make_shared<SimAdc>();
  } else if (config.ads1115.enable) {
    auto ads1115 =
        std::make_shared<ADS1115>(config.ads1115.i2c_bus, DefaultADSAddress);
    ads1115->init(toAdsGain(config.ads1115.gain),
                  toAdsDataRate(config.ads1115.data_rate),
                  static_cast<ADS1115::Channel>(config.ads1115.channel),
                  config.ads1115.external_gain);
    sensors.adc = ads1115;
  }

  if (config.sim.imu) {
    if (config.icm20948.enable || config.mid360.enable) {
      log::warn("sim.imu overrides the enabled hardware IMU.");
    }
    auto sim = std::make_shared<SimImu>();
    sim->startSampling();
    sensors.imu = sim;
  } else if (config.icm20948.enable && !sensors.imu) {
    auto icm20948 =
        std::make_shared<ICM20948>(config.icm20948.i2c_bus, ICM20948_ADDR0);
    icm20948->init();
    icm20948->calibrate();
    icm20948->startSampling();
    sensors.imu = icm20948;
  }

  if (config.sim.camera) {
    if (config.camera.enable) {
      log::warn("sim.camera overrides the enabled camera.");
    }
    auto sim = std::make_shared<SimCamera>();
    sim->startSampling();
    sensors.camera = sim;
  } else if (config.camera.enable) {
    log::info("Using camera pipeline: {}", config.camera.pipeline);
    auto camera = std::make_shared<OpenCvCamera>(
        std::string(config.camera.pipeline),
        config.camera.encoding == "jpeg" ? CameraFrameEncoding::Jpeg
                                         : CameraFrameEncoding::Raw);
    camera->startSampling();
    sensors.camera = camera;
  }

  return sensors;
}

} // namespace msensor
