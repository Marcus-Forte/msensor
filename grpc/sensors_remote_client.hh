#pragma once

#include <grpcpp/channel.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "imu.grpc.pb.h"
#include "lidar.grpc.pb.h"
#include "msensor/interface/CallbackSlot.hh"
#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"

namespace msensor {

/// Per-stream queue depth the client asks the server for (see
/// `max_queued_samples` in the stream requests). 0 selects the server default.
struct RemoteClientOptions {
  uint32_t scan_queue_size = 0; ///< Scans the server may hold for us.
  uint32_t imu_queue_size = 0;  ///< IMU samples the server may hold for us.
};

/**
 * @brief This class connects to a SensorService and provides methods to get
 * sensor data remotely.
 *
 */
class SensorsRemoteClient : public msensor::ILidar, public msensor::IImu {
public:
  explicit SensorsRemoteClient(const std::string &remote_ip,
                               RemoteClientOptions options = {});
  virtual ~SensorsRemoteClient();
  /// Establish the gRPC channel and prepare internal state.
  void init() override;
  /// Start background threads that pull data from the server.
  void start();
  /// Stop background readers and tear down the connection.
  void stop();
  void startSampling() override;
  void stopSampling() override;

  /// Register the single consumer of LiDAR scans.
  void setScanCallback(ScanCallback callback) override;
  /// Register the single consumer of IMU samples.
  void setImuCallback(ImuCallback callback) override;

  /// LiDAR scans missing from the stream so far, counted from gaps in
  /// `header.sequence_number` (a jump back is a restart, not a loss; a
  /// reconnect starts a new stream). Thread-safe; monotonic.
  uint64_t lostScans() const noexcept {
    return lost_scans_.load(std::memory_order_relaxed);
  }
  /// IMU samples missing from the stream so far; see lostScans().
  uint64_t lostImuSamples() const noexcept {
    return lost_imu_samples_.load(std::memory_order_relaxed);
  }

private:
  std::string remote_ip_;
  RemoteClientOptions options_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<sensors::LidarService::Stub> lidar_stub_;
  std::unique_ptr<sensors::ImuService::Stub> imu_stub_;

  msensor::CallbackSlot<msensor::Scan3DI> scan_callback_;
  msensor::CallbackSlot<msensor::IMUData> imu_callback_;

  std::atomic<uint64_t> lost_scans_{0};
  std::atomic<uint64_t> lost_imu_samples_{0};

  // Active stream contexts, so stop() can cancel blocked reads.
  std::mutex ctx_m_;
  grpc::ClientContext *lidar_ctx_ = nullptr;
  grpc::ClientContext *imu_ctx_ = nullptr;

  // Last: joined before the members above are destroyed.
  std::jthread read_thread_;
  std::jthread imu_reader_thread_;
};

} // namespace msensor

#ifndef MSENSOR_NO_LEGACY_NAMES
// Legacy global names, for downstream code. Define MSENSOR_NO_LEGACY_NAMES to
// drop them.
using msensor::SensorsRemoteClient;
#endif
