#pragma once

#include "interface/ILidar.hh"
#include "lidar.grpc.pb.h"

/**
 * @brief Implements the LiDAR gRPC service using the callback API.
 *
 * CallbackService provides reactor-based async handling, allowing
 * independent reads and writes on bidirectional streams without threads.
 */
class LidarServiceImpl : public sensors::LidarService::CallbackService {
public:
  LidarServiceImpl(std::shared_ptr<msensor::ILidar> lidar);

  grpc::ServerWriteReactor<sensors::PointCloud3> *
  getLidarScan(grpc::CallbackServerContext *context,
               const sensors::LidarStreamRequest *request) override;

  grpc::ServerBidiReactor<sensors::SubSampledLidarStreamRequest,
                          sensors::PointCloud3> *
  getSubSampledLidarScan(grpc::CallbackServerContext *context) override;

private:
  std::shared_ptr<msensor::ILidar> lidar_;
};
