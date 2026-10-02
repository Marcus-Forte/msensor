#pragma once

#include "lidar.grpc.pb.h"
#include "msensor/interface/ILidar.hh"
#include "task_runner.hh"

/**
 * @brief Implements the LiDAR gRPC service using the callback API.
 *
 * Each stream subscribes to the lidar's scan hub, so any number of clients can
 * stream at once and a slow client only drops its own scans. Streams are
 * event-driven: no thread is held while waiting for data.
 */
class LidarServiceImpl : public sensors::LidarService::CallbackService {
public:
  LidarServiceImpl(std::shared_ptr<msensor::ILidar> lidar);

  grpc::ServerWriteReactor<sensors::PointCloud3> *
  getLidarScan(grpc::CallbackServerContext *context,
               const sensors::LidarStreamRequest *request) override;

private:
  std::shared_ptr<msensor::ILidar> lidar_;
  TaskRunner runner_;
};
