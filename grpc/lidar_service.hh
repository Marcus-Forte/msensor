#pragma once

#include <atomic>
#include <memory>

#include "lidar.grpc.pb.h"
#include "msensor/interface/ILidar.hh"

namespace msensor {

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

} // namespace msensor

#ifndef MSENSOR_NO_LEGACY_NAMES
// Legacy global names, for downstream code. Define MSENSOR_NO_LEGACY_NAMES to
// drop them.
using msensor::LidarServiceImpl;
#endif
