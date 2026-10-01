#include <condition_variable>
#include <google/protobuf/empty.pb.h>
#include <grpcpp/client_context.h>
#include <grpcpp/grpcpp.h>
#include <mutex>

#include "msensor/conversions/conversions.hh"
#include "sensors_remote_client.hh"

constexpr int g_connectionRecoverDelayMs = 1000;

SensorsRemoteClient::SensorsRemoteClient(const std::string &remote_ip)
    : remote_ip_(remote_ip) {

  channel_ = grpc::CreateChannel(remote_ip, grpc::InsecureChannelCredentials());
  lidar_stub_ = sensors::LidarService::NewStub(channel_);
  imu_stub_ = sensors::ImuService::NewStub(channel_);
}

void SensorsRemoteClient::init() {}
void SensorsRemoteClient::startSampling() {}
void SensorsRemoteClient::stopSampling() {}

SensorsRemoteClient::~SensorsRemoteClient() { stop(); }

namespace {

void interruptibleSleep(std::stop_token st, std::chrono::milliseconds d) {
  std::mutex m;
  std::condition_variable_any cv;
  std::unique_lock lock(m);
  cv.wait_for(lock, st, d, [] { return false; });
}

/// Opens a stream, forwards every message to `on_msg`, and reopens the stream
/// after a delay when it ends. `active` always points at the live context (or
/// is null), under `m`, so stop() can cancel a blocked Read().
template <class Msg, class Open, class OnMsg>
void readLoop(std::stop_token st, std::mutex &m, grpc::ClientContext *&active,
              const char *name, Open open, OnMsg on_msg) {
  while (!st.stop_requested()) {
    grpc::ClientContext context;
    {
      std::lock_guard lock(m);
      if (st.stop_requested()) {
        return;
      }
      active = &context;
    }

    {
      auto reader = open(&context);
      Msg msg;
      while (reader->Read(&msg)) {
        on_msg(msg);
      }
    }

    {
      std::lock_guard lock(m);
      active = nullptr;
    }

    if (st.stop_requested()) {
      return;
    }
    std::cout << "Unable to read remote " << name << "." << std::endl;
    interruptibleSleep(st,
                       std::chrono::milliseconds(g_connectionRecoverDelayMs));
  }
}

} // namespace

void SensorsRemoteClient::start() {
  read_thread_ = std::jthread([this](std::stop_token st) {
    const sensors::LidarStreamRequest request;
    readLoop<sensors::PointCloud3>(
        st, ctx_m_, lidar_ctx_, "lidar",
        [&](grpc::ClientContext *ctx) {
          return lidar_stub_->getLidarScan(ctx, request);
        },
        [&](const sensors::PointCloud3 &msg) {
          scan_hub_.publish(fromProtobuf(msg));
        });
  });

  imu_reader_thread_ = std::jthread([this](std::stop_token st) {
    const sensors::ImuStreamRequest request;
    readLoop<sensors::IMUData>(
        st, ctx_m_, imu_ctx_, "imu",
        [&](grpc::ClientContext *ctx) {
          return imu_stub_->getImuData(ctx, request);
        },
        [&](const sensors::IMUData &msg) {
          imu_hub_.publish(fromProtobuf(msg));
        });
  });
}

void SensorsRemoteClient::stop() {
  read_thread_.request_stop();
  imu_reader_thread_.request_stop();
  {
    // Unblock readers waiting on a silent stream.
    std::lock_guard lock(ctx_m_);
    if (lidar_ctx_) {
      lidar_ctx_->TryCancel();
    }
    if (imu_ctx_) {
      imu_ctx_->TryCancel();
    }
  }
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (imu_reader_thread_.joinable()) {
    imu_reader_thread_.join();
  }
}
