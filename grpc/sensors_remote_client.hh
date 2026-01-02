#pragma once

#include <boost/lockfree/spsc_queue.hpp>
#include <grpcpp/channel.h>
#include <memory>
#include <thread>

#include "interface/IImu.hh"
#include "interface/ILidar.hh"
#include "sensors.grpc.pb.h"

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

  /// Pop the next LiDAR scan received over gRPC.
  std::shared_ptr<msensor::Scan3DI> getScan() override;
  /// Pop the next IMU sample received over gRPC.
  std::optional<msensor::IMUData> getImuData() override;

private:
  std::string remote_ip_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<sensors::SensorService::Stub> service_stub_;

  std::jthread read_thread_;
  std::jthread imu_reader_thread_;
  std::unique_ptr<grpc::ClientContext> context_;

  boost::lockfree::spsc_queue<std::shared_ptr<msensor::Scan3DI>> scan_queue_;
  boost::lockfree::spsc_queue<msensor::IMUData> imu_queue_;
};