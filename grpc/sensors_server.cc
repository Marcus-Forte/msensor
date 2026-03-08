#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>

#include "sensors_server.hh"

SensorsServer::SensorsServer(std::shared_ptr<msensor::IAdc> adc,
                             std::shared_ptr<msensor::ICamera> camera,
                             std::shared_ptr<msensor::IImu> imu,
                             std::shared_ptr<msensor::ILidar> lidar)
    : lidar_service_(lidar), imu_service_(imu), camera_service_(camera),
      adc_service_(adc) {}

void SensorsServer::start() {

  grpc::ServerBuilder builder;
  builder.AddListeningPort("0.0.0.0:50051",
                           ::grpc::InsecureServerCredentials());
  builder.RegisterService(&lidar_service_);
  builder.RegisterService(&imu_service_);
  builder.RegisterService(&camera_service_);
  builder.RegisterService(&adc_service_);

  server_ = builder.BuildAndStart();
  std::cout << "Listening..." << std::endl;
}

void SensorsServer::stop() { server_->Shutdown(); }
