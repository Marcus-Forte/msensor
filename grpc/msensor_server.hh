#pragma once

#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>

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
                std::shared_ptr<msensor::ILidar> lidar = nullptr);

  void start();
  void stop();

private:
  LidarServiceImpl lidar_service_;
  ImuServiceImpl imu_service_;
  CameraServiceImpl camera_service_;
  AdcServiceImpl adc_service_;
  std::unique_ptr<grpc::Server> server_;
};