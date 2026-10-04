#include <chrono>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>
#include <utility>

#include "msensor_server.hh"

SensorsServer::SensorsServer(std::shared_ptr<msensor::IAdc> adc,
                             std::shared_ptr<msensor::ICamera> camera,
                             std::shared_ptr<msensor::IImu> imu,
                             std::shared_ptr<msensor::ILidar> lidar,
                             std::string listen_address)
    : lidar_service_(lidar), imu_service_(imu), camera_service_(camera),
      adc_service_(adc), listen_address_(std::move(listen_address)) {}

void SensorsServer::start() {

  grpc::ServerBuilder builder;
  builder.AddListeningPort(listen_address_,
                           ::grpc::InsecureServerCredentials());
  builder.RegisterService(&lidar_service_);
  builder.RegisterService(&imu_service_);
  builder.RegisterService(&camera_service_);
  builder.RegisterService(&adc_service_);

  server_ = builder.BuildAndStart();
  std::cout << "Listening on " << listen_address_ << "..." << std::endl;
}

void SensorsServer::stop() {
  if (server_) {
    // Give in-flight streams a moment to finish so a client sees a clean
    // end-of-stream instead of a dropped connection.
    server_->Shutdown(std::chrono::system_clock::now() +
                      std::chrono::seconds(1));
  }
}
