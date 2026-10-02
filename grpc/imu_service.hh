#pragma once

#include <atomic>
#include <memory>

#include "imu.grpc.pb.h"
#include "msensor/interface/IImu.hh"

/**
 * @brief Implements the IMU gRPC service.
 *
 * One client per stream: a second concurrent stream is rejected with
 * RESOURCE_EXHAUSTED. Samples are pushed by the driver's callback, so the
 * service holds no thread waiting for data.
 */
class ImuServiceImpl : public sensors::ImuService::CallbackService {
public:
  explicit ImuServiceImpl(std::shared_ptr<msensor::IImu> imu);

  grpc::ServerWriteReactor<sensors::IMUData> *
  getImuData(grpc::CallbackServerContext *context,
             const sensors::ImuStreamRequest *request) override;

private:
  std::shared_ptr<msensor::IImu> imu_;
  std::atomic<bool> in_use_{false};
};
