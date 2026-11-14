#include <chrono>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>
#include <stdexcept>
#include <utility>

#include "msensor/log/log.hh"
#include "msensor_server.hh"

namespace msensor {

SensorsServer::SensorsServer(Sensors sensors, ServerOptions options)
    : lidar_service_(std::move(sensors.lidar)),
      imu_service_(std::move(sensors.imu)),
      camera_service_(std::move(sensors.camera), options.camera_jpeg_quality),
      adc_service_(std::move(sensors.adc)),
      listen_address_(std::move(options.listen_address)) {}

// Legacy form: forwards to the constructor above.
SensorsServer::SensorsServer(std::shared_ptr<IAdc> adc,
                             std::shared_ptr<ICamera> camera,
                             std::shared_ptr<IImu> imu,
                             std::shared_ptr<ILidar> lidar,
                             std::string listen_address)
    : SensorsServer(Sensors{std::move(adc), std::move(camera), std::move(imu),
                            std::move(lidar)},
                    ServerOptions{.listen_address = std::move(listen_address)}) {}

void SensorsServer::start() {

  grpc::ServerBuilder builder;
  builder.AddListeningPort(listen_address_,
                           ::grpc::InsecureServerCredentials());
  builder.RegisterService(&lidar_service_);
  builder.RegisterService(&imu_service_);
  builder.RegisterService(&camera_service_);
  builder.RegisterService(&adc_service_);

  server_ = builder.BuildAndStart();
  if (!server_) {
    throw std::runtime_error("Unable to start gRPC server on " +
                             listen_address_);
  }
  log::info("Listening on {}...", listen_address_);
}

void SensorsServer::wait() {
  if (server_) {
    server_->Wait();
  }
}

void SensorsServer::stop() {
  if (server_) {
    // Give in-flight streams a moment to finish so a client sees a clean
    // end-of-stream instead of a dropped connection.
    server_->Shutdown(std::chrono::system_clock::now() +
                      std::chrono::seconds(1));
  }
}

} // namespace msensor
