#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>

#include "sensors_server.hh"

SensorsServer::SensorsServer(std::shared_ptr<IAdc> adc, std::shared_ptr<ICamera> camera,
                               std::shared_ptr<IImu> imu,
                               std::shared_ptr<ILidar> lidar)
    : scan_service_(adc, camera, imu, lidar) {}

void SensorsServer::start() {

  grpc::ServerBuilder builder;
  builder.AddListeningPort("0.0.0.0:50051",
                           ::grpc::InsecureServerCredentials());
  builder.RegisterService(&scan_service_);

  server_ = builder.BuildAndStart();
  std::cout << "Listening..." << std::endl;
}

void SensorsServer::stop() { server_->Shutdown(); }
