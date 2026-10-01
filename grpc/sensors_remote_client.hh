#pragma once

#include <grpcpp/channel.h>
#include <memory>
#include <mutex>
#include <thread>

#include "imu.grpc.pb.h"
#include "lidar.grpc.pb.h"
#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"

/**
 * @brief This class connects to a SensorService and provides methods to get
 * sensor data remotely.
 *
 */
class SensorsRemoteClient : public msensor::ILidar, public msensor::IImu {
public:
  SensorsRemoteClient(const std::string &remote_ip);
  virtual ~SensorsRemoteClient();
  /// Establish the gRPC channel and prepare internal queues.
  void init() override;
  /// Start background threads that pull data from the server.
  void start();
  /// Stop background readers and tear down the connection.
  void stop();
  void startSampling() override;
  void stopSampling() override;

  /// Hub publishing LiDAR scans received over gRPC.
  msensor::SensorHub<msensor::Scan3DI> &scans() override { return scan_hub_; }
  /// Hub publishing IMU samples received over gRPC.
  msensor::SensorHub<msensor::IMUData> &imu() override { return imu_hub_; }

private:
  std::string remote_ip_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<sensors::LidarService::Stub> lidar_stub_;
  std::unique_ptr<sensors::ImuService::Stub> imu_stub_;

  msensor::SensorHub<msensor::Scan3DI> scan_hub_;
  msensor::SensorHub<msensor::IMUData> imu_hub_;

  // Active stream contexts, so stop() can cancel blocked reads.
  std::mutex ctx_m_;
  grpc::ClientContext *lidar_ctx_ = nullptr;
  grpc::ClientContext *imu_ctx_ = nullptr;

  // Last: joined before the members above are destroyed.
  std::jthread read_thread_;
  std::jthread imu_reader_thread_;
};