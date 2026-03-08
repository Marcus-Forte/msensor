#pragma once

#include "interface/ILidar.hh"
#include "lidar.grpc.pb.h"

/**
 * @brief Implements the LiDAR gRPC service.
 */
class LidarServiceImpl : public sensors::LidarService::Service {
public:
  LidarServiceImpl(std::shared_ptr<msensor::ILidar> lidar);

  ::grpc::Status
  getLidarScan(::grpc::ServerContext *context,
               const ::sensors::LidarStreamRequest *request,
               ::grpc::ServerWriter<sensors::PointCloud3> *writer) override;

private:
  std::shared_ptr<msensor::ILidar> lidar_;
};
