#pragma once

#include "interface/IAdc.hh"

#include <cstdint>
#include <optional>

namespace msensor {

/// Minimal ADS1115 driver backing the generic ADC interface.
class ADS1115 final : public IAdc {
public:
  enum class Gain : uint16_t {
    PLUS_MINUS_6_144 = 0,
    PLUS_MINUS_4_096,
    PLUS_MINUS_2_048,
    PLUS_MINUS_1_024,
    PLUS_MINUS_0_512,
    PLUS_MINUS_0_256,
  };

  enum class DataRate : uint16_t {
    SPS_8 = 0,
    SPS_16,
    SPS_32,
    SPS_64,
    SPS_128,
    SPS_250,
    SPS_475,
    SPS_860,
  };

  enum class Channel : uint16_t {
    SINGLE_0 = 0,
    SINGLE_1 = 1,
    SINGLE_2 = 2,
    SINGLE_3 = 3,
  };

  /**
   * @brief Construct an ADS1115 wrapper.
   *
   * @param i2cBus Linux I2C bus index.
   * @param address I2C address of the ADC.
   */
  ADS1115(int i2cBus = 1, uint8_t address = 0x48);
  ~ADS1115() = default;

  /**
   * @brief Configure ADC gain, sampling rate and input channel.
   *
   * @return true on success, false otherwise.
   */
  bool init(Gain gain, DataRate rate, Channel channel,
            float external_gain = 1.0f);

  /// Read the configured channel as a single-ended voltage sample.
  std::optional<AdcSample> readSingleEnded() const override;

private:
  /// Write a 16-bit word to the given register address.
  int16_t write_(uint8_t reg_address, uint16_t data) const;
  /// Read a 16-bit word from the given register address.
  int16_t read_(uint8_t reg_address) const;

  /// Retrieve the latest conversion result.
  int16_t readConversion() const;
  /// Convert raw ADC counts to volts according to configured gain.
  float convertRawToVoltage(int16_t raw) const;

  const int i2c_device_;
  const int i2c_ads_address_;
  int i2c_device_fd_;
  Gain gain_;
  float external_gain_;
};

} // namespace msensor
