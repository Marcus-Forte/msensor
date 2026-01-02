#include "adc/ADS1115.hh"
#include "sensors_server.hh"

#include <cstdlib>
#include <cstring>
#include <getopt.h>
#include <iostream>
#include <optional>
#include <thread>

namespace {
constexpr uint32_t kDefaultSampleRateHz = 100;
constexpr int kDefaultChannel = 0;
constexpr int kDefaultI2cBus = 1;
constexpr uint8_t kDefaultAddress = 0x48;

void print_usage() {
  std::cout << "Usage: adc_publisher [-b <i2c bus>] [-c <channel 0-3>]"
            << std::endl;
}

std::optional<msensor::AdcChannel> channelFromIndex(int channel) {
  using AdcChannel = msensor::AdcChannel;
  switch (channel) {
  case 0:
    return AdcChannel::CHANNEL_0;
  case 1:
    return AdcChannel::CHANNEL_1;
  case 2:
    return AdcChannel::CHANNEL_2;
  case 3:
    return AdcChannel::CHANNEL_3;
  default:
    return std::nullopt;
  }
}
} // namespace

int main(int argc, char **argv) {
  int opt;
  int bus = kDefaultI2cBus;
  int channel = kDefaultChannel;

  while ((opt = getopt(argc, argv, "b:c:h")) != -1) {
    switch (opt) {
    case 'b':
      bus = std::strtol(optarg, nullptr, 0);
      break;
    case 'c':
      channel = std::strtol(optarg, nullptr, 0);
      break;
    case 'h':
    default:
      print_usage();
      return 0;
    }
  }

  const auto target_channel = channelFromIndex(channel);
  if (!target_channel) {
    std::cerr << "Invalid channel " << channel << ". Channel must be 0..3."
              << std::endl;
    return 1;
  }

  auto adc = std::make_shared<msensor::ADS1115>(bus, kDefaultAddress);
  auto res = adc->init(msensor::ADS1115::Gain::PLUS_MINUS_6_144,
                      msensor::ADS1115::DataRate::SPS_8,
                      static_cast<msensor::ADS1115::Channel>(channel));
  if (!res) {
    std::cerr << "Failed to initialize ADS1115 on I2C bus " << bus
              << " address 0x" << std::hex << static_cast<int>(kDefaultAddress)
              << std::dec << std::endl;
  }
  SensorsServer server(adc, nullptr, nullptr, nullptr);
  server.start();

  while (true) {
    if (auto sample = adc->readSingleEnded()) {
      std::cout << "ADC ch" << channel << " = " << sample->voltage << " V"
                << " @ " << sample->timestamp << std::endl;
      constexpr float ExternalGain =
          (10.0f + 5.1f) / 5.1f; // 10k and 5.1k resistors
      sample->voltage *= ExternalGain;

      std::cout << "Vin = " << sample->voltage << std::endl;

    } else {
      std::cerr << "ADC read failure on channel " << channel << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  server.stop();
  return 0;
}
