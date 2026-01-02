#pragma once

#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>

#include "sensors_service.hh"

/**
 * @brief This class manages the gRPC server and provides methods to publish
 * data.
 *
 */
class SensorsServer {
public:
  SensorsServer(std::shared_ptr<IAdc> adc = nullptr,
                std::shared_ptr<ICamera> camera = nullptr,
                std::shared_ptr<IImu> imu = nullptr,
                std::shared_ptr<ILidar> lidar = nullptr);

  void start();
  void stop();

private:
  ScanService scan_service_;
  std::unique_ptr<grpc::Server> server_;
};