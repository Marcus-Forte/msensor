#include "msensor/lidar/mid360.hh"

#include <algorithm>
#include <future>
#include <livox_lidar_def.h>

#include <iostream>
#include <string>

#include "livox_lidar_api.h"
#include "msensor/interface/ILidar.hh"

namespace msensor {

constexpr size_t g_max_queue_elements = 50;
constexpr size_t g_max_scan_points_per_packet = 96;

namespace {

void convertEthPacketInto(const LivoxLidarEthernetPacket *eth_packet,
                          unsigned int data_pts,
                          pcl::PointCloud<pcl::PointXYZI> &dest) {
  const auto *data_ = reinterpret_cast<const LivoxLidarCartesianHighRawPoint *>(
      eth_packet->data);
  const size_t count =
      std::min(static_cast<size_t>(data_pts), g_max_scan_points_per_packet);
  const size_t offset = dest.size();
  dest.resize(offset + count);
  for (size_t i = 0; i < count; ++i) {
    dest[offset + i].x = static_cast<float>(data_[i].x) / 1000.0F;
    dest[offset + i].y = static_cast<float>(data_[i].y) / 1000.0F;
    dest[offset + i].z = static_cast<float>(data_[i].z) / 1000.0F;
    dest[offset + i].intensity = data_[i].reflectivity;
  }
}
} // namespace

Mid360::Mid360(std::string config, size_t accumulate_scan_count)
    : config_{std::move(config)}, accumulate_scan_count_(accumulate_scan_count),
      scan_queue_(g_max_queue_elements), imu_queue_(g_max_queue_elements),
      scan_count_(0) {}

void Mid360::startSampling() {
  if (!LivoxLidarSdkStart()) {
    throw std::runtime_error("Unable to initialize Mid360!");
  }
};
void Mid360::stopSampling() { /* \todo */ };

void Mid360::setMode(Mode mode) {
  // Wake up is actually idle...
  const LivoxLidarWorkMode _mode = mode == Mode::Normal
                                       ? LivoxLidarWorkMode::kLivoxLidarNormal
                                       : LivoxLidarWorkMode::kLivoxLidarWakeUp;

  std::promise<void> promise_complete;
  const auto future = promise_complete.get_future();

  SetLivoxLidarWorkMode(
      connection_handle_, _mode,
      [](livox_status status, uint32_t handle,
         LivoxLidarAsyncControlResponse *response, void *client_data) {
        printf("WorkModeCallback, status:%u, handle:%u, ret_code:%u, "
               "error_key:%u\n",
               status, handle, response->ret_code, response->error_key);
        auto *promise = static_cast<std::promise<void> *>(client_data);
        promise->set_value();
      },
      &promise_complete);

  if (future.wait_for(std::chrono::milliseconds(1000)) ==
      std::future_status::timeout) {
    throw std::runtime_error("Unable to set mode!");
  }
};

void Mid360::setScanPattern(ScanPattern pattern) const {
  LivoxLidarScanPattern scan_pattern;
  if (pattern == ScanPattern::Repetitive) {
    scan_pattern = kLivoxLidarScanPatternRepetive;
  } else if (pattern == ScanPattern::NonRepetitive) {
    scan_pattern = kLivoxLidarScanPatternNoneRepetive;
  } else {
    scan_pattern = kLivoxLidarScanPatternRepetiveLowFrameRate;
  }

  std::promise<void> promise_complete;
  const auto future = promise_complete.get_future();

  SetLivoxLidarScanPattern(
      connection_handle_, scan_pattern,
      [](livox_status status, uint32_t handle,
         LivoxLidarAsyncControlResponse *response, void *client_data) {
        printf("SetLivoxLidarScanPattern, status:%u, handle:%u, ret_code:%u, "
               "error_key:%u\n",
               status, handle, response->ret_code, response->error_key);
        auto *promise = static_cast<std::promise<void> *>(client_data);
        promise->set_value();
      },
      &promise_complete);

  if (future.wait_for(std::chrono::milliseconds(1000)) ==
      std::future_status::timeout) {
    throw std::runtime_error("Unable to set scan pattern!");
  }
}

void Mid360::init() {
  std::cout << "config:  " << config_ << std::endl;
  if (!LivoxLidarSdkInit(config_.c_str())) {
    LivoxLidarSdkUninit();
    std::cout << "Livox-SDK init fail!" << std::endl;
    throw std::runtime_error("Unable to initialize Mid360!");
  }

  std::promise<int> promise_complete;
  auto future = promise_complete.get_future();

  SetLivoxLidarInfoChangeCallback(
      [](const uint32_t handle, const LivoxLidarInfo *info, void *client_data) {
        if (info == nullptr) {
          return;
        }

        auto *this_ = reinterpret_cast<decltype(this)>(client_data);

        std::cout << "Lidar IP: " << info->lidar_ip << std::endl;
        std::cout << "DevType: " << info->dev_type << std::endl;
        std::cout << "SN: " << info->sn << std::endl;
        std::cout << "handle: " << std::to_string(handle) << std::endl;

        auto *promise = static_cast<std::promise<int> *>(client_data);
        promise->set_value(handle);
      },
      &promise_complete);

  if (future.wait_for(std::chrono::milliseconds(10000)) ==
      std::future_status::timeout) {
    throw std::runtime_error("Unable to get Lidar Info scan!");
  }
  connection_handle_ = future.get();
  std::cout << "connection_handle_: " << connection_handle_ << std::endl;

  SetLivoxLidarImuDataCallback(
      [](const uint32_t handle, const uint8_t dev_type,
         LivoxLidarEthernetPacket *data, void *client_data) {
        if (data == nullptr) {
          return;
        }

        static uint32_t sequence_number = 0;
        auto *this_ = reinterpret_cast<decltype(this)>(client_data);
        auto *data_ = reinterpret_cast<LivoxLidarImuRawPoint *>(data->data);

        auto imu_data = IMUData(
            {*reinterpret_cast<uint64_t *>(data->timestamp), sequence_number++},
            data_->acc_x, data_->acc_y, data_->acc_z, data_->gyro_x,
            data_->gyro_y, data_->gyro_z);
        this_->imu_queue_.push(imu_data);
      },
      this);

  SetLivoxLidarPointCloudCallBack(
      [](const uint32_t handle, const uint8_t dev_type,
         LivoxLidarEthernetPacket *data, void *client_data) {
        if (data == nullptr) {
          return;
        }
        auto *this_ = reinterpret_cast<decltype(this)>(client_data);

        static uint32_t sequence_number = 0;
        if (!this_->accumulated_pointcloud_data_) {
          this_->accumulated_pointcloud_data_ = std::make_shared<Scan3DI>();
          this_->accumulated_pointcloud_data_->points->reserve(
              this_->accumulate_scan_count_ * g_max_scan_points_per_packet);
          this_->accumulated_pointcloud_data_->header =
              Header{*reinterpret_cast<uint64_t *>(data->timestamp),
                     sequence_number++};
        }

        convertEthPacketInto(data, data->dot_num,
                             *this_->accumulated_pointcloud_data_->points);

        if (++this_->scan_count_ % this_->accumulate_scan_count_ == 0) {

          this_->scan_queue_.push(this_->accumulated_pointcloud_data_);
          this_->accumulated_pointcloud_data_.reset();
        }
      },
      this);
}

std::shared_ptr<Scan3DI> Mid360::getScan() {
  if (scan_queue_.empty()) {
    return nullptr;
  }

  auto last = std::move(scan_queue_.front());
  scan_queue_.pop();
  return last;
}

std::optional<IMUData> Mid360::getImuData() {
  if (imu_queue_.empty()) {
    return std::nullopt;
  }
  auto last = std::move(imu_queue_.front());
  imu_queue_.pop();
  return last;
}
} // namespace msensor