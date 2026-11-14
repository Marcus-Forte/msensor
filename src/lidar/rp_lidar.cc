#include "msensor/lidar/rp_lidar.hh"
#include <cmath>
#include <chrono>
#include <filesystem>
#include <format>
#include <numbers>
#include <stdexcept>
#include <string>
#include <thread>

#include "msensor/log/log.hh"
#include "msensor/timing/timing.hh"

namespace msensor {

namespace {
constexpr uint32_t g_baudRate = 115200;
constexpr auto kNoScanBackoff = std::chrono::milliseconds(10);

Scan3DI toScan3D(const sl_lidar_response_measurement_node_hq_t *nodes,
                 int count, uint32_t sequence_number) {
  Scan3DI scan;
  scan.points.reserve(count);
  for (int pos = 0; pos < (int)count; ++pos) {
    if (nodes[pos].quality < 40)
      continue;

    const float angle =
        static_cast<float>(nodes[pos].angle_z_q14) *
        std::numbers::pi_v<float> / 32768.0F;
    const float dist_m =
        static_cast<float>(nodes[pos].dist_mm_q2) / 4000.0F;
    const float x = -std::cos(angle) * dist_m;
    const float y = std::sin(angle) * dist_m;
    scan.points.emplace_back(x, y, 0, 0);
  }
  scan.header = {Header{timing::getNowNs(), sequence_number}};

  return scan;
}
} // namespace

RPLidar::RPLidar(const std::string &serial_port) : serial_port_(serial_port) {
  auto drv = sl::createLidarDriver();
  if (!drv || !*drv) {
    throw std::runtime_error("Unable to create the RPLidar driver");
  }
  drv_.reset(*drv);

  auto channel = sl::createSerialPortChannel(serial_port, g_baudRate);
  if (!channel || !*channel) {
    // drv_ is released by its unique_ptr.
    throw std::runtime_error("Unable to create the serial channel for " +
                             serial_port);
  }
  channel_.reset(*channel);
}

RPLidar::~RPLidar() {
  // The sampling thread uses the driver: join it first.
  stopSampling();
  if (drv_ && drv_->isConnected()) {
    drv_->stop();
    drv_->setMotorSpeed(0); // stops the motor
    drv_->disconnect();
  }
  // The driver must go before the channel it was connected to.
  drv_.reset();
  channel_.reset();
}

void RPLidar::init() {
  // The SDK's connect() reports success even when the serial port could not be
  // opened (a shadowed result variable in AsyncTransceiver::openChannelAndBind),
  // and later calls then dereference a null channel. Reject a missing device
  // up front, and treat a device that does not answer as a failed connection.
  if (!std::filesystem::exists(serial_port_)) {
    throw std::runtime_error("LiDAR device does not exist: " + serial_port_);
  }
  if (!SL_IS_OK(drv_->connect(channel_.get()))) {
    throw std::runtime_error("Unable to connect to LiDAR");
  }
  sl_lidar_response_device_info_t devinfo;
  auto op_result = drv_->getDeviceInfo(devinfo);
  if (!SL_IS_OK(op_result)) {
    drv_->disconnect();
    throw std::runtime_error("LiDAR on " + serial_port_ +
                             " did not answer the device info request");
  }
  {
    std::string serial;
    for (int pos = 0; pos < 16; ++pos) {
      serial += std::format("{:02X}", devinfo.serialnum[pos]);
    }
    log::info("SLAMTEC LIDAR S/N: {} Firmware Ver: {}.{} Hardware Rev: {}",
              serial, devinfo.firmware_version >> 8,
              devinfo.firmware_version & 0xFF, (int)devinfo.hardware_version);
  }

  sl::LidarMotorInfo motorinfo;
  drv_->getMotorInfo(motorinfo);
  log::info("Motor info: desired speed: {}, min_speed: {}, "
            "max_speed: {}, ctrl_support: {}",
            motorinfo.desired_speed, motorinfo.min_speed, motorinfo.max_speed,
            static_cast<int>(motorinfo.motorCtrlSupport));

  drv_->setMotorSpeed(0);
  drv_->startScan(0, 1);
}

std::optional<Scan3DI> RPLidar::grabScan() {

  sl_lidar_response_measurement_node_hq_t nodes[8192];
  size_t count = sizeof(nodes) / sizeof(nodes[0]);

  auto result = drv_->grabScanDataHq(nodes, count, 5);
  if (SL_IS_OK(result)) {
    drv_->ascendScanData(nodes, count); // AKA Reorder
    return toScan3D(nodes, count, sequence_number_++);
  } else {
    return std::nullopt;
  }
}

void RPLidar::startSampling() {
  sampler_.startFreeRunning([this]() -> std::chrono::nanoseconds {
    if (auto scan = grabScan()) {
      callback_.emit(std::move(*scan));
      return std::chrono::nanoseconds::zero();
    }
    return kNoScanBackoff;
  });
}

void RPLidar::stopSampling() {
  sampler_.stop();
}

void RPLidar::setScanCallback(ScanCallback callback) {
  callback_.setCallback(std::move(callback));
}

void RPLidar::setMotorRPM(unsigned int rpm) { drv_->setMotorSpeed(rpm); }

} // namespace msensor