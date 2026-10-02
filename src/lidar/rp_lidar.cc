#include "msensor/lidar/rp_lidar.hh"
#include <cmath>
#include <chrono>
#include <format>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <thread>

#include "msensor/timing/timing.hh"

namespace msensor {

constexpr uint32_t g_baudRate = 115200;

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

RPLidar::RPLidar(const std::string &serial_port) {

  drv_ = *sl::createLidarDriver();

  if (!drv_) {
    fprintf(stderr, "insufficent memory, exit\n");
    exit(-2);
  }

  channel_ = *sl::createSerialPortChannel(serial_port, g_baudRate);
}
RPLidar::~RPLidar() = default;

void RPLidar::init() {

  if (!SL_IS_OK((drv_)->connect(channel_))) {
    throw std::runtime_error("Unable to connect to LiDAR");
  }
  sl_lidar_response_device_info_t devinfo;
  auto op_result = drv_->getDeviceInfo(devinfo);
  if (SL_IS_OK(op_result)) {
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos) {
      printf("%02X", devinfo.serialnum[pos]);
    }
    std::cout << std::format("Firmware Ver: {}.{}"
                             "Hardware Rev: {}",
                             devinfo.firmware_version >> 8,
                             devinfo.firmware_version & 0xFF,
                             (int)devinfo.hardware_version)
              << std::endl;
  }

  sl::LidarMotorInfo motorinfo;
  drv_->getMotorInfo(motorinfo);
  std::cout << std::format("Motor info: desired speed: {}, min_speed: {}, "
                           "max_speed: {}, ctrl_support: {}",
                           motorinfo.desired_speed, motorinfo.min_speed,
                           motorinfo.max_speed,
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
  if (thread_.joinable()) {
    return;
  }
  thread_ = std::jthread([this](std::stop_token st) { run(st); });
}

void RPLidar::stopSampling() {
  if (!thread_.joinable()) {
    return;
  }
  thread_.request_stop();
  thread_.join();
}

void RPLidar::setScanCallback(ScanCallback callback) {
  callback_.setCallback(std::move(callback));
}

void RPLidar::run(std::stop_token st) {
  while (!st.stop_requested()) {
    if (auto scan = grabScan()) {
      callback_.emit(std::move(*scan));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void RPLidar::setMotorRPM(unsigned int rpm) { drv_->setMotorSpeed(rpm); }

} // namespace msensor