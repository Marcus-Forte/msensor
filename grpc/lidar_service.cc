
#include "lidar_service.hh"
#include "conversions.hh"
#include <pcl/point_types.h>

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


grpc::Status LidarServiceImpl::getSubSampledLidarScan(::grpc::ServerContext* context, ::grpc::ServerReaderWriter< ::sensors::PointCloud3, ::sensors::SubSampledLidarStreamRequest>* stream) {

  static bool s_client_connected = false;

  if (!lidar_) {
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, "Lidar not available");
  }

  if (s_client_connected) {
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client per stream supported");
  }

  std::cout << "Start subsampled Lidar scan stream." << std::endl;

  s_client_connected = true;

  sensors::SubSampledLidarStreamRequest request;
  while (!context->IsCancelled()) {
    if (const auto scan = lidar_->getScan()) {
      /// TODO: apply subsample filter
      if(stream->Read(&request)) {
        std::cout << "Received subsample request with voxel size: " << request.voxel_size() << std::endl;

      }

      if (const auto scan = lidar_->getScan()) {
      stream->Write(toGRPC(scan));
    }
    }
  }
  std::cout << "Ending subsampled Lidar scan stream." << std::endl;
  s_client_connected = false;

  return ::grpc::Status::OK;
}

