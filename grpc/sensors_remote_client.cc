#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <google/protobuf/empty.pb.h>
#include <grpcpp/client_context.h>
#include <grpcpp/grpcpp.h>
#include <mutex>

#include "msensor/conversions/lidar_imu_conversions.hh"
#include "msensor/log/log.hh"
#include "sensors_remote_client.hh"
#include "sequence_gap_counter.hh"

namespace msensor {

constexpr int g_connectionRecoverDelayMs = 1000;

SensorsRemoteClient::SensorsRemoteClient(const std::string &remote_ip,
                                         RemoteClientOptions options)
    : remote_ip_(remote_ip), options_(options) {

  channel_ = grpc::CreateChannel(remote_ip, grpc::InsecureChannelCredentials());
  lidar_stub_ = sensors::LidarService::NewStub(channel_);
  imu_stub_ = sensors::ImuService::NewStub(channel_);
}

void SensorsRemoteClient::init() {}
void SensorsRemoteClient::startSampling() {}
void SensorsRemoteClient::stopSampling() {}

void SensorsRemoteClient::setScanCallback(ScanCallback callback) {
  scan_callback_.setCallback(std::move(callback));
}

void SensorsRemoteClient::setImuCallback(ImuCallback callback) {
  imu_callback_.setCallback(std::move(callback));
}

SensorsRemoteClient::~SensorsRemoteClient() { stop(); }

bool SensorsRemoteClient::linkUp() const {
  // try_to_connect = false: only observe; the reader threads drive connecting.
  return channel_->GetState(false) == GRPC_CHANNEL_READY;
}

namespace {

int64_t steadyNowNs() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void interruptibleSleep(std::stop_token st, std::chrono::milliseconds d) {
  std::mutex m;
  std::condition_variable_any cv;
  std::unique_lock lock(m);
  cv.wait_for(lock, st, d, [] { return false; });
}

/// Loss bookkeeping of one stream, on its reader thread: counts the gaps in
/// the sequence numbers into `total` and warns about them, at most once per
/// second. It only observes; it never touches the samples.
class LossTracker {
public:
  LossTracker(const char *name, std::atomic<uint64_t> &total)
      : name_(name), total_(total) {}

  /// A new connection starts a new stream: its first number means nothing.
  void reconnected() noexcept { gap_.reset(); }

  void observe(uint32_t sequence_number) {
    const uint32_t lost = gap_.observe(sequence_number);
    if (lost == 0) {
      return;
    }
    const uint64_t total =
        total_.fetch_add(lost, std::memory_order_relaxed) + lost;
    unreported_ += lost;
    const auto now = std::chrono::steady_clock::now();
    if (warned_ && now - last_warn_ < std::chrono::seconds(1)) {
      return;
    }
    log::warn("Remote {} stream: {} sample(s) lost (sequence number gap, "
              "now at {}); {} lost since the client started.",
              name_, unreported_, sequence_number, total);
    unreported_ = 0;
    last_warn_ = now;
    warned_ = true;
  }

private:
  const char *name_;
  std::atomic<uint64_t> &total_;
  SequenceGapCounter gap_;
  uint64_t unreported_ = 0; // lost since the last warning
  std::chrono::steady_clock::time_point last_warn_;
  bool warned_ = false;
};

/// Opens a stream, forwards every message to `on_msg`, and reopens the stream
/// after a delay when it ends. `active` always points at the live context (or
/// is null), under `m`, so stop() can cancel a blocked Read(). Stream closure
/// is treated as temporary so a running consumer survives a server restart or
/// a finite playback reaching its end.
template <class Msg, class Open, class OnConnect, class OnMsg>
void readLoop(std::stop_token st, std::mutex &m, grpc::ClientContext *&active,
              const char *name, Open open, OnConnect on_connect, OnMsg on_msg) {
  while (!st.stop_requested()) {
    grpc::ClientContext context;
    {
      std::lock_guard lock(m);
      if (st.stop_requested()) {
        return;
      }
      active = &context;
    }

    grpc::Status status;
    on_connect();
    {
      auto reader = open(&context);
      Msg msg;
      while (reader->Read(&msg)) {
        on_msg(msg);
      }
      status = reader->Finish();
    }

    {
      std::lock_guard lock(m);
      active = nullptr;
    }

    if (st.stop_requested()) {
      return;
    }
    if (status.ok()) {
      log::warn("Remote {} stream ended; reconnecting.", name);
    } else {
      log::warn("Unable to read remote {}: {}", name, status.error_message());
    }
    interruptibleSleep(st,
                       std::chrono::milliseconds(g_connectionRecoverDelayMs));
  }
}

} // namespace

std::optional<std::chrono::milliseconds>
SensorsRemoteClient::ageSince(const std::atomic<int64_t> &stamp_ns) noexcept {
  const int64_t stamp = stamp_ns.load(std::memory_order_relaxed);
  if (stamp == kNever) {
    return std::nullopt;
  }
  const int64_t age_ns = std::max<int64_t>(0, steadyNowNs() - stamp);
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::nanoseconds(age_ns));
}

void SensorsRemoteClient::start() {
  read_thread_ = std::jthread([this](std::stop_token st) {
    sensors::LidarStreamRequest request;
    request.set_max_queued_samples(options_.scan_queue_size);
    LossTracker loss("lidar", lost_scans_);
    readLoop<sensors::PointCloud3>(
        st, ctx_m_, lidar_ctx_, "lidar",
        [&](grpc::ClientContext *ctx) {
          return lidar_stub_->getLidarScan(ctx, request);
        },
        [&] { loss.reconnected(); },
        [&](const sensors::PointCloud3 &msg) {
          last_scan_ns_.store(steadyNowNs(), std::memory_order_relaxed);
          Scan3DI scan = fromProtobuf(msg);
          loss.observe(scan.header.sequence_number);
          scan_callback_.emit(std::move(scan));
        });
  });

  imu_reader_thread_ = std::jthread([this](std::stop_token st) {
    sensors::ImuStreamRequest request;
    request.set_max_queued_samples(options_.imu_queue_size);
    LossTracker loss("imu", lost_imu_samples_);
    readLoop<sensors::IMUData>(
        st, ctx_m_, imu_ctx_, "imu",
        [&](grpc::ClientContext *ctx) {
          return imu_stub_->getImuData(ctx, request);
        },
        [&] { loss.reconnected(); },
        [&](const sensors::IMUData &msg) {
          last_imu_ns_.store(steadyNowNs(), std::memory_order_relaxed);
          IMUData sample = fromProtobuf(msg);
          loss.observe(sample.header.sequence_number);
          imu_callback_.emit(std::move(sample));
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

} // namespace msensor
