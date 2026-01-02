#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>

#include "sensors_server.hh"

SensorsServer::SensorsServer() = default;

void SensorsServer::start() {

  grpc::ServerBuilder builder;
  builder.AddListeningPort("0.0.0.0:50051",
                           ::grpc::InsecureServerCredentials());
  builder.RegisterService(&scan_service_);

  server_ = builder.BuildAndStart();
  std::cout << "Listening..." << std::endl;
}

void SensorsServer::stop() { server_->Shutdown(); }

void SensorsServer::publishScan(const std::shared_ptr<msensor::Scan3DI> &scan) {
  scan_service_.putScan(scan);
}

void SensorsServer::publishImu(msensor::IMUData imu_data) {
  scan_service_.putImuData(imu_data);
}

void SensorsServer::publishAdc(msensor::AdcSample adc_data) {
  scan_service_.putAdcData(adc_data);
}

void SensorsServer::publishCameraFrame(const cv::Mat &image) {
  scan_service_.putImage(image);
}
