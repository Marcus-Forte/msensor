#pragma once

#include <cstddef>
#include <memory>

#include "imu.grpc.pb.h"
#include "msensor/bus/sensor_bus.hh"
#include "msensor/interface/IImu.hh"
#include "sensor_stream.hh"

namespace msensor {

/**
 * @brief Implements the IMU gRPC service.
 *
 * Any number of clients up to `max_clients` may stream at once; one more is
 * rejected with RESOURCE_EXHAUSTED. Each is a subscriber of the sensor's bus
 * topic. Samples are pushed by the driver's callback, so the
 * service holds no thread waiting for data.
 */
class ImuServiceImpl : public sensors::ImuService::CallbackService {
public:
  /// Serves `imu` through a private bus, with the default client limit.
  explicit ImuServiceImpl(std::shared_ptr<msensor::IImu> imu);

  /// Serves the imu topic of a shared bus to at most `max_clients` clients
  /// at once.
  ImuServiceImpl(std::shared_ptr<SensorBus> bus, size_t max_clients);

  grpc::ServerWriteReactor<sensors::IMUData> *
  getImuData(grpc::CallbackServerContext *context,
             const sensors::ImuStreamRequest *request) override;

private:
  std::shared_ptr<SensorBus> bus_;
  ClientLimit limit_;
};

} // namespace msensor

#ifndef MSENSOR_NO_LEGACY_NAMES
// Legacy global names, for downstream code. Define MSENSOR_NO_LEGACY_NAMES to
// drop them.
using msensor::ImuServiceImpl;
#endif
