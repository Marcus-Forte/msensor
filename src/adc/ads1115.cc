#include "adc/ADS1115.hh"

#include <fcntl.h>
extern "C" {
#include <i2c/smbus.h>
}
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "timing/timing.hh"

#include <format>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

namespace msensor {

namespace {
constexpr uint8_t REG_CONVERSION = 0x00;
constexpr uint8_t REG_CONFIG = 0x01;
constexpr uint16_t CONFIG_OS_SINGLE = 0x8000;
constexpr uint16_t CONFIG_MODE_SINGLE = 0x0100;
constexpr uint16_t CONFIG_MODE_CONTINUOUS = 0x0000;
constexpr uint16_t CONFIG_COMP_QUEUE_DISABLE = 0x0003;

uint16_t channelBits(ADS1115::Channel channel) {
  switch (channel) {
  case ADS1115::Channel::SINGLE_0:
    return 0x4000;
  case ADS1115::Channel::SINGLE_1:
    return 0x5000;
  case ADS1115::Channel::SINGLE_2:
    return 0x6000;
  case ADS1115::Channel::SINGLE_3:
    return 0x7000;
  default:
    return 0x4000;
  }
}

uint16_t gainBits(ADS1115::Gain gain) {
  return (static_cast<uint16_t>(gain) & 0x7) << 9;
}

float gainRange(ADS1115::Gain gain) {
  switch (gain) {
  case ADS1115::Gain::PLUS_MINUS_6_144:
    return 6.144f;
  case ADS1115::Gain::PLUS_MINUS_4_096:
    return 4.096f;
  case ADS1115::Gain::PLUS_MINUS_2_048:
    return 2.048f;
  case ADS1115::Gain::PLUS_MINUS_1_024:
    return 1.024f;
  case ADS1115::Gain::PLUS_MINUS_0_512:
    return 0.512f;
  case ADS1115::Gain::PLUS_MINUS_0_256:
    return 0.256f;
  }
  return 2.048f;
}

uint16_t dataRateBits(ADS1115::DataRate rate) {
  return (static_cast<uint16_t>(rate) & 0x7) << 5;
}

inline uint16_t hostToBe(uint16_t v) {
  return static_cast<uint16_t>((v >> 8) | (v << 8));
}
inline uint16_t beToHost(uint16_t v) { return hostToBe(v); }

} // namespace

ADS1115::ADS1115(int i2c_device, uint8_t i2c_ads_address)
    : i2c_device_(i2c_device), i2c_ads_address_(i2c_ads_address) {

  const std::string i2c_device_file = "/dev/i2c-" + std::to_string(i2c_device);
  i2c_device_fd_ = open(i2c_device_file.c_str(), O_RDWR);

  if (i2c_device_fd_ < 0) {
    throw std::runtime_error("unable to open I2C (system) device: " +
                             std::to_string(i2c_device));
  }

  if (ioctl(i2c_device_fd_, I2C_SLAVE, i2c_ads_address_) < 0) {
    throw std::runtime_error("unable to open IMU device: " +
                             std::to_string(i2c_ads_address_)); // to hex?
  }
}

std::optional<AdcSample> ADS1115::readSingleEnded() const {
  const auto raw = readConversion();
  return AdcSample{convertRawToVoltage(raw), timing::getNowUs()};
}

bool ADS1115::init(Gain gain, DataRate rate, Channel channel,
                   float external_gain) {
  gain_ = gain;
  external_gain_ = external_gain;

  uint16_t config = 0;
  config |= channelBits(channel);
  config |= gainBits(gain_);
  config |= dataRateBits(rate);
  config |= CONFIG_MODE_CONTINUOUS;
  // config |= CONFIG_OS_SINGLE;
  config |= CONFIG_COMP_QUEUE_DISABLE;
  std::cout << std::format("ADS1115 config: 0x{:04X}\n", config);
  int16_t res = write_(REG_CONFIG, config);

  std::cout << std::format("res: {}\n", res);

  uint16_t readback = read_(REG_CONFIG);
  std::cout << std::format("Post Config ADS1115 readback config: 0x{:04X}\n",
                           readback);

  return res == 0;
}

int16_t ADS1115::readConversion() const {
  const auto raw = read_(REG_CONVERSION);
  return raw;
}

float ADS1115::convertRawToVoltage(int16_t raw) const {
  float range = gainRange(gain_);
  return (static_cast<float>(raw) / 32768.0f) * range;
}

inline int16_t ADS1115::write_(uint8_t reg_address, uint16_t data) const {
  // smbus word writes are little-endian; ADS1115 expects MSB first.
  return i2c_smbus_write_word_data(i2c_device_fd_, reg_address, hostToBe(data));
}

inline int16_t ADS1115::read_(uint8_t reg_address) const {
  const auto raw = i2c_smbus_read_word_data(i2c_device_fd_, reg_address);
  return beToHost(static_cast<uint16_t>(raw));
}

} //