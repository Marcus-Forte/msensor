#pragma once

#include <cstddef>
#include <memory>

#include "lidar.grpc.pb.h"
#include "msensor/bus/sensor_bus.hh"
#include "msensor/interface/ILidar.hh"
#include "sensor_stream.hh"

namespace msensor {

/**
 * @brief Implements the LiDAR gRPC service.
 *
 * Any number of clients up to `max_clients` may stream at once; one more is
 * rejected with RESOURCE_EXHAUSTED. Each is a subscriber of the sensor's bus
 * topic. Scans are pushed by the driver's callback, so the service
 * holds no thread waiting for data.
 */
class LidarServiceImpl : public sensors::LidarService::CallbackService {
public:
  /// Serves `lidar` through a private bus, with the default client limit.
  explicit LidarServiceImpl(std::shared_ptr<msensor::ILidar> lidar);

  /// Serves the lidar topic of a shared bus to at most `max_clients` clients
  /// at once.
  LidarServiceImpl(std::shared_ptr<SensorBus> bus, size_t max_clients);

  grpc::ServerWriteReactor<sensors::PointCloud3> *
  getLidarScan(grpc::CallbackServerContext *context,
               const sensors::LidarStreamRequest *request) override;

private:
  std::shared_ptr<SensorBus> bus_;
  ClientLimit limit_;
};

} // namespace msensor

#ifndef MSENSOR_NO_LEGACY_NAMES
// Legacy global names, for downstream code. Define MSENSOR_NO_LEGACY_NAMES to
// drop them.
using msensor::LidarServiceImpl;
#endif
