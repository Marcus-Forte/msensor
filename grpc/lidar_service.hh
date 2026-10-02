#pragma once

#include <atomic>
#include <memory>

#include "lidar.grpc.pb.h"
#include "msensor/interface/ILidar.hh"

/**
 * @brief Implements the LiDAR gRPC service.
 *
 * One client per stream: a second concurrent stream is rejected with
 * RESOURCE_EXHAUSTED. Scans are pushed by the driver's callback, so the service
 * holds no thread waiting for data.
 */
class LidarServiceImpl : public sensors::LidarService::CallbackService {
public:
  explicit LidarServiceImpl(std::shared_ptr<msensor::ILidar> lidar);

  grpc::ServerWriteReactor<sensors::PointCloud3> *
  getLidarScan(grpc::CallbackServerContext *context,
               const sensors::LidarStreamRequest *request) override;

private:
  std::shared_ptr<msensor::ILidar> lidar_;
  std::atomic<bool> in_use_{false};
};
