
#include "lidar_service.hh"
#include "conversions.hh"

LidarServiceImpl::LidarServiceImpl(std::shared_ptr<msensor::ILidar> lidar)
    : lidar_(lidar) {}

grpc::Status LidarServiceImpl::getLidarScan(
    ::grpc::ServerContext *context,
    const ::sensors::LidarStreamRequest *request,
    ::grpc::ServerWriter<sensors::PointCloud3> *writer) {

  static bool s_client_connected = false;

  if (!lidar_) {
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, "Lidar not available");
  }

  if (s_client_connected) {
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client per stream supported");
  }

  std::cout << "Start Lidar scan stream." << std::endl;

  s_client_connected = true;

  while (!context->IsCancelled()) {
    if (const auto scan = lidar_->getScan()) {
      writer->Write(toGRPC(scan));
    }
  }
  std::cout << "Ending Lidar scan stream." << std::endl;
  s_client_connected = false;

  return ::grpc::Status::OK;
}
