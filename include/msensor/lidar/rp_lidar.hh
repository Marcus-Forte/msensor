#pragma once

#include <memory>
#include <optional>
#include <string>

#include "msensor/interface/CallbackSlot.hh"
#include "msensor/interface/SamplingThread.hh"
#include "msensor/interface/ILidar.hh"
#include "sl_lidar_driver.h"

namespace msensor {

/**
 * @brief Wrapper around the RPLidar SDK providing the ILidar interface.
 */
class RPLidar : public ILidar {
public:
  /// Create the SDK driver and serial channel (the device is opened by
  /// `init()`). Throws std::runtime_error if either cannot be created.
  explicit RPLidar(const std::string &serial_port);
  /// Stops sampling; if connected, stops the scan and the motor and
  /// disconnects; then releases the driver and the channel. Safe if `init()`
  /// was never called or threw.
  ~RPLidar() override;
  RPLidar(const RPLidar &) = delete;
  RPLidar &operator=(const RPLidar &) = delete;

  void init() override;
  /// Start publishing scans.
  void startSampling() override;
  /// Stop publishing scans.
  void stopSampling() override;
  /// Register the single scan consumer.
  void setScanCallback(ScanCallback callback) override;
  /// Configure motor speed in RPM.
  void setMotorRPM(unsigned int rpm);

private:
  /// Acquire a single scan from the sensor, or nullopt on timeout.
  std::optional<Scan3DI> grabScan();


  const std::string serial_port_;
  // Declared channel first: the driver is destroyed before its channel.
  std::unique_ptr<sl::IChannel> channel_;
  std::unique_ptr<sl::ILidarDriver> drv_;
  uint32_t sequence_number_ = 0;
  CallbackSlot<Scan3DI> callback_;
  SamplingThread sampler_; // last: joined before the members above are destroyed
};

} // namespace msensor
