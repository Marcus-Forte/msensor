#include <google/protobuf/empty.pb.h>
#include <grpcpp/client_context.h>
#include <grpcpp/grpcpp.h>
#include <thread>

#include "conversions.hh"
#include "sensors_remote_client.hh"

constexpr int g_idleTimeMs = 5;
constexpr size_t g_maxLidarSamples = 100;
constexpr size_t g_maxImuSamples = 200;
constexpr int g_connectionRecoverDelayMs = 1000;

SensorsRemoteClient::SensorsRemoteClient(const std::string &remote_ip)
    : remote_ip_(remote_ip), scan_queue_(g_maxLidarSamples),
      imu_queue_(g_maxImuSamples) {

  channel_ = grpc::CreateChannel(remote_ip, grpc::InsecureChannelCredentials());
  service_stub_ = sensors::SensorService::NewStub(channel_);
}

void SensorsRemoteClient::init() {}
void SensorsRemoteClient::startSampling() {}
void SensorsRemoteClient::stopSampling() {}

SensorsRemoteClient::~SensorsRemoteClient() { stop(); }

std::shared_ptr<msensor::Scan3DI> SensorsRemoteClient::getScan() {

  if (!scan_queue_.empty()) {
    auto pointcloud = scan_queue_.front();
    scan_queue_.pop();
    return pointcloud;
  }

  return nullptr;
}

std::optional<msensor::IMUData> SensorsRemoteClient::getImuData() {

  if (!imu_queue_.empty()) {
    const auto imu_data = imu_queue_.front();
    imu_queue_.pop();
    return imu_data;
  }

  return std::nullopt;
}

void SensorsRemoteClient::start() {

  read_thread_ = std::jthread([&](std::stop_token stop_token) {
    auto service_context_ = std::make_unique<grpc::ClientContext>();
    sensors::SensorStreamRequest request;

    auto reader = service_stub_->getLidarScan(service_context_.get(), request);

    sensors::PointCloud3 msg;

    while (!stop_token.stop_requested()) {
      if (!reader->Read(&msg)) {
        std::cout << "Unable to read remote lidar." << std::endl;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(g_connectionRecoverDelayMs));
        service_context_ = std::make_unique<grpc::ClientContext>();
        reader = service_stub_->getLidarScan(service_context_.get(),
                                             request); // retry
      } else {
        scan_queue_.push(fromGRPC(msg));
      }
    }
  });

  imu_reader_thread_ = std::jthread([&](std::stop_token stop_token) {
    auto service_context_ = std::make_unique<grpc::ClientContext>();
    sensors::SensorStreamRequest request;

    auto imu_reader =
        service_stub_->getImuData(service_context_.get(), request);
    sensors::IMUData msg;

    while (!stop_token.stop_requested()) {

      if (!imu_reader->Read(&msg)) {
        std::cout << "Unable to read remote imu." << std::endl;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(g_connectionRecoverDelayMs));
        service_context_ = std::make_unique<grpc::ClientContext>();
        imu_reader = service_stub_->getImuData(service_context_.get(), request);
      } else {
        imu_queue_.push(fromGRPC(msg));
      }
    }
  });
}

void SensorsRemoteClient::stop() {
  read_thread_.request_stop();
  imu_reader_thread_.request_stop();

  /// \todo why does it not work in unit test?
  read_thread_.join();
  imu_reader_thread_.join();
}
