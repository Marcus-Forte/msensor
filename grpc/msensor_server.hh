#pragma once

#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>

#include <memory>
#include <string>

#include "adc_service.hh"
#include "camera_service.hh"
#include "imu_service.hh"
#include "lidar_service.hh"

/**
 * @brief This class manages the gRPC server and provides methods to publish
 * data.
 *
 */
class SensorsServer {
public:
  SensorsServer(std::shared_ptr<msensor::IAdc> adc = nullptr,
                std::shared_ptr<msensor::ICamera> camera = nullptr,
                std::shared_ptr<msensor::IImu> imu = nullptr,
                std::shared_ptr<msensor::ILidar> lidar = nullptr,
                std::string listen_address = "0.0.0.0:50051");

  void start();
  void stop();

private:
  LidarServiceImpl lidar_service_;
  ImuServiceImpl imu_service_;
  CameraServiceImpl camera_service_;
  AdcServiceImpl adc_service_;
  std::string listen_address_;
  std::unique_ptr<grpc::Server> server_;
};
