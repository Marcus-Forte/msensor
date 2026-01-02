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
  SensorsServer();

  void start();
  void stop();

  void publishScan(const std::shared_ptr<msensor::Scan3DI> &scan);
  void publishImu(msensor::IMUData data);
  void publishAdc(msensor::AdcSample data);
  void publishCameraFrame(const cv::Mat &image);

private:
  ScanService scan_service_;
  std::unique_ptr<grpc::Server> server_;
};