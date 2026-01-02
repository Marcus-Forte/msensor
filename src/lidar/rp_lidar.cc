#include "lidar/rp_lidar.hh"
#include <format>
#include <iostream>
#include <stdexcept>

#include "timing/timing.hh"

namespace msensor {

constexpr uint32_t g_baudRate = 115200;

std::shared_ptr<Scan3DI>
toScan3D(const sl_lidar_response_measurement_node_hq_t *nodes, int count) {

  std::shared_ptr<Scan3DI> scan = std::make_shared<Scan3DI>();
  scan->points = std::make_shared<PointCloud3I>();
  scan->points->reserve(count);
  int idx = 0;
  for (int pos = 0; pos < (int)count; ++pos) {
    if (nodes[pos].quality < 40)
      continue;

    float angle_in_pi = (nodes[pos].angle_z_q14 * M_PI_2) / 16384.f;
    const float dist_m = nodes[pos].dist_mm_q2 / 4000.0f;
    float x = -cos(angle_in_pi) * dist_m;
    float y = sin(angle_in_pi) * dist_m;
    scan->points->emplace_back(x, y, 0, 0);
  }
  scan->timestamp = timing::getNowUs();

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

std::shared_ptr<Scan3DI> RPLidar::getScan() {

  sl_lidar_response_measurement_node_hq_t nodes[8192];
  size_t count = sizeof(nodes) / sizeof(nodes[0]);

  auto result = drv_->grabScanDataHq(nodes, count, 5);
  if (SL_IS_OK(result)) {
    drv_->ascendScanData(nodes, count); // AKA Reorder
    return toScan3D(nodes, count);
  } else {
    return nullptr;
  }
}

void RPLidar::setMotorRPM(unsigned int rpm) { drv_->setMotorSpeed(rpm); }

} // namespace msensor