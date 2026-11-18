#include "camera.grpc.pb.h"
#include "imu.grpc.pb.h"
#include "lidar.grpc.pb.h"
#include "msensor/bus/sensor_bus.hh"
#include "msensor/camera/sim_camera.hh"
#include "msensor/imu/sim_imu.hh"
#include "msensor/interface/CallbackSlot.hh"
#include "msensor/lidar/sim_lidar.hh"
#include "msensor/recorder/bus_recorder.hh"
#include "msensor/recorder/scan_player.hh"
#include "msensor_server.hh"
#include "recording.pb.h"
#include "sensors_remote_client.hh"
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <grpcpp/grpcpp.h>
#include <gtest/gtest.h>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

class TestClientServer : public ::testing::Test {
public:
  void SetUp() override {
    lidar = std::make_shared<msensor::SimLidar>(true);
    imu = std::make_shared<msensor::SimImu>();
    server = std::make_shared<msensor::SensorsServer>(
        msensor::Sensors{.imu = imu, .lidar = lidar});
    client = std::make_shared<msensor::SensorsRemoteClient>("localhost:50051");
  }

  void TearDown() override {
    client->stop();
    server->stop();
    lidar->stopSampling();
    imu->stopSampling();
  }

protected:
  std::shared_ptr<msensor::SimLidar> lidar;
  std::shared_ptr<msensor::SimImu> imu;
  std::shared_ptr<msensor::SensorsServer> server;
  std::shared_ptr<msensor::SensorsRemoteClient> client;
};

TEST_F(TestClientServer, StreamsScansAndImuToClient) {
  std::mutex m;
  std::condition_variable cv;
  msensor::Scan3DI scan;
  msensor::IMUData imu_sample;
  bool got_scan = false;
  bool got_imu = false;

  client->setScanCallback([&](const msensor::Scan3DI &sample) {
    std::lock_guard lock(m);
    scan = sample;
    got_scan = true;
    cv.notify_all();
  });
  client->setImuCallback([&](const msensor::IMUData &sample) {
    std::lock_guard lock(m);
    imu_sample = sample;
    got_imu = true;
    cv.notify_all();
  });

  server->start();
  lidar->startSampling();
  imu->startSampling();
  client->start();

  {
    std::unique_lock lock(m);
    ASSERT_TRUE(cv.wait_for(lock, 5s, [&] { return got_scan && got_imu; }));
  }
  EXPECT_EQ(scan.points.size(), 2000u);
  EXPECT_GT(scan.header.timestamp, 0u);
}

TEST_F(TestClientServer, SecondClientIsServedToo) {
  server->start();
  lidar->startSampling();

  auto channel = grpc::CreateChannel("localhost:50051",
                                     grpc::InsecureChannelCredentials());
  auto stub = sensors::LidarService::NewStub(channel);
  const sensors::LidarStreamRequest request;

  grpc::ClientContext first_context;
  auto first = stub->getLidarScan(&first_context, request);
  grpc::ClientContext second_context;
  auto second = stub->getLidarScan(&second_context, request);
  sensors::PointCloud3 a, b;
  EXPECT_TRUE(first->Read(&a));
  EXPECT_TRUE(second->Read(&b));
  EXPECT_EQ(a.x_size(), 2000);
  EXPECT_EQ(b.x_size(), 2000);

  first_context.TryCancel();
  second_context.TryCancel();
  first->Finish();
  second->Finish();
}

TEST(SensorsServerTest, ClientBeyondTheLimitIsRejectedAndASlotFreesUp) {
  const std::string address = "127.0.0.1:50078";
  auto lidar = std::make_shared<msensor::SimLidar>(true);
  lidar->startSampling();
  msensor::SensorsServer server(
      msensor::Sensors{.lidar = lidar},
      {.listen_address = address, .max_clients_per_stream = 2});
  server.start();
  auto stub = sensors::LidarService::NewStub(
      grpc::CreateChannel(address, grpc::InsecureChannelCredentials()));
  const sensors::LidarStreamRequest request;
  sensors::PointCloud3 msg;

  grpc::ClientContext c1, c2, c3;
  auto s1 = stub->getLidarScan(&c1, request);
  auto s2 = stub->getLidarScan(&c2, request);
  ASSERT_TRUE(s1->Read(&msg));
  ASSERT_TRUE(s2->Read(&msg));

  auto s3 = stub->getLidarScan(&c3, request);
  EXPECT_FALSE(s3->Read(&msg));
  EXPECT_EQ(s3->Finish().error_code(), grpc::StatusCode::RESOURCE_EXHAUSTED);

  // When one client leaves, its slot is free again.
  c1.TryCancel();
  s1->Finish();
  bool served = false;
  for (int attempt = 0; attempt < 50 && !served; ++attempt) {
    grpc::ClientContext retry;
    auto s = stub->getLidarScan(&retry, request);
    served = s->Read(&msg);
    retry.TryCancel();
    s->Finish();
    if (!served) {
      std::this_thread::sleep_for(50ms);
    }
  }
  EXPECT_TRUE(served);

  c2.TryCancel();
  s2->Finish();
  server.stop();
  lidar->stopSampling();
}

TEST(SensorsServerTest, ClientLimitIsPerStream) {
  const std::string address = "127.0.0.1:50079";
  auto lidar = std::make_shared<msensor::SimLidar>(true);
  auto imu = std::make_shared<msensor::SimImu>();
  lidar->startSampling();
  imu->startSampling();
  msensor::SensorsServer server(
      msensor::Sensors{.imu = imu, .lidar = lidar},
      {.listen_address = address, .max_clients_per_stream = 1});
  server.start();
  auto channel =
      grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
  auto lidar_stub = sensors::LidarService::NewStub(channel);
  auto imu_stub = sensors::ImuService::NewStub(channel);

  grpc::ClientContext lc, ic;
  auto ls = lidar_stub->getLidarScan(&lc, sensors::LidarStreamRequest{});
  auto is = imu_stub->getImuData(&ic, sensors::ImuStreamRequest{});
  sensors::PointCloud3 cloud;
  sensors::IMUData sample;
  EXPECT_TRUE(ls->Read(&cloud));
  EXPECT_TRUE(is->Read(&sample)); // a full LiDAR stream does not block the IMU
  lc.TryCancel();
  ic.TryCancel();
  ls->Finish();
  is->Finish();
  server.stop();
  lidar->stopSampling();
  imu->stopSampling();
}

// A bus shared with another consumer: the server serves a client while a
// second, in-process subscriber reads the same samples.
TEST(SensorsServerTest, SharedBusServesClientsAndOtherSubscribers) {
  const std::string address = "127.0.0.1:50080";
  auto imu = std::make_shared<msensor::SimImu>();
  auto bus = std::make_shared<msensor::SensorBus>(msensor::Sensors{.imu = imu});
  auto local = bus->imu()->subscribe(1000);
  msensor::SensorsServer server({.listen_address = address}, bus);
  server.start();
  imu->startSampling();

  msensor::SensorsRemoteClient client(address);
  std::mutex m;
  std::condition_variable cv;
  int received = 0;
  client.setImuCallback([&](const msensor::IMUData &) {
    std::lock_guard lock(m);
    ++received;
    cv.notify_all();
  });
  client.start();
  {
    std::unique_lock lock(m);
    EXPECT_TRUE(cv.wait_for(lock, 5s, [&] { return received >= 5; }));
  }
  EXPECT_GE(local->size(), 5U);
  client.stop();
  server.stop();
  imu->stopSampling();
}

TEST(SensorsServerTest, NullSharedBusServesUnavailable) {
  msensor::SensorsServer server({.listen_address = "127.0.0.1:50081"}, nullptr);
  server.start();
  server.stop();
}

TEST_F(TestClientServer, MissingSensorsReportUnavailableWithoutCrashing) {
  auto empty_server =
      std::make_shared<msensor::SensorsServer>(msensor::Sensors{});
  empty_server->start();
  msensor::SensorsRemoteClient c("localhost:50051");
  c.start(); // streams end with UNAVAILABLE, client retries quietly
  std::this_thread::sleep_for(200ms);
  c.stop();
  empty_server->stop();
}

namespace {

/// Emits exactly the samples a test pushes, like a driver's sampling thread.
class FeedSensor : public msensor::ILidar, public msensor::IImu {
public:
  void init() override {}
  void startSampling() override {}
  void stopSampling() override {}
  void setScanCallback(ScanCallback callback) override {
    scans_.setCallback(std::move(callback));
  }
  void setImuCallback(ImuCallback callback) override {
    imus_.setCallback(std::move(callback));
  }
  bool hasScanConsumer() const { return scans_.hasCallback(); }
  bool hasImuConsumer() const { return imus_.hasCallback(); }
  void push(msensor::Scan3DI scan) { scans_.emit(std::move(scan)); }
  void push(msensor::IMUData imu) { imus_.emit(std::move(imu)); }

private:
  msensor::CallbackSlot<msensor::Scan3DI> scans_;
  msensor::CallbackSlot<msensor::IMUData> imus_;
};

msensor::Scan3DI makeScan(uint32_t seq, size_t points) {
  msensor::Scan3DI scan;
  scan.header = {1'000'000'000ULL + seq, seq};
  for (size_t i = 0; i < points; ++i) {
    const float f = static_cast<float>(i) * 0.01F + static_cast<float>(seq);
    scan.points.push_back({f, -f, f * 0.5F, static_cast<float>(i % 256)});
  }
  return scan;
}

} // namespace

// Scans pass the server's conversion, the wire and the client's conversion
// without any change: the client sees bit-identical points.
TEST(ClientServerContent, ScansArriveBitExact) {
  const std::string address = "127.0.0.1:50071";
  auto feed = std::make_shared<FeedSensor>();
  msensor::SensorsServer server({.lidar = feed}, {.listen_address = address});
  server.start();
  msensor::SensorsRemoteClient client(address);

  std::mutex m;
  std::condition_variable cv;
  std::vector<msensor::Scan3DI> received;
  client.setScanCallback([&](msensor::Scan3DI scan) {
    std::lock_guard lock(m);
    received.push_back(std::move(scan));
    cv.notify_all();
  });
  client.start();
  for (int i = 0; i < 500 && !feed->hasScanConsumer(); ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(feed->hasScanConsumer());

  // Odd sizes exercise the scalar tails of the vectorised conversions.
  const std::vector<size_t> sizes{1, 7, 9600, 333, 0, 100003};
  for (uint32_t i = 0; i < sizes.size(); ++i) {
    const auto scan = makeScan(i, sizes[i]);
    feed->push(scan); // one at a time: latest-wins would drop queued scans
    std::unique_lock lock(m);
    ASSERT_TRUE(cv.wait_for(lock, 5s, [&] { return received.size() > i; }));
    const auto &got = received[i];
    EXPECT_EQ(got.header.timestamp, scan.header.timestamp);
    EXPECT_EQ(got.header.sequence_number, scan.header.sequence_number);
    ASSERT_EQ(got.points.size(), scan.points.size());
    if (!scan.points.empty()) { // memcmp(nullptr, ...) is undefined
      EXPECT_EQ(std::memcmp(got.points.data(), scan.points.data(),
                            scan.points.size() * sizeof(msensor::Point3I)),
                0)
          << "scan " << i;
    }
  }
  client.stop();
  server.stop();
}

namespace {

/// Emits a back-to-back burst of `burst` IMU samples numbered 0..burst-1 once
/// the client's stream is attached, while the client asks for `queue_size`.
/// Returns the sequence numbers received (bounded wait for the last one).
std::vector<uint32_t> imuBurst(const std::string &address, uint32_t queue_size,
                               uint32_t burst) {
  auto feed = std::make_shared<FeedSensor>();
  msensor::SensorsServer server({.imu = feed}, {.listen_address = address});
  server.start();
  msensor::SensorsRemoteClient client(address, {.imu_queue_size = queue_size});

  std::mutex m;
  std::condition_variable cv;
  std::vector<uint32_t> seqs;
  client.setImuCallback([&](const msensor::IMUData &imu) {
    std::lock_guard lock(m);
    seqs.push_back(imu.header.sequence_number);
    cv.notify_all();
  });
  client.start();
  for (int i = 0; i < 500 && !feed->hasImuConsumer(); ++i) {
    std::this_thread::sleep_for(10ms); // polling for the stream to attach
  }
  EXPECT_TRUE(feed->hasImuConsumer());

  for (uint32_t i = 0; i < burst; ++i) {
    msensor::IMUData imu{};
    imu.header = {1'000'000'000ULL + i, i};
    feed->push(imu);
  }
  {
    // The newest sample is always delivered, so wait for it.
    std::unique_lock lock(m);
    EXPECT_TRUE(cv.wait_for(
        lock, 10s, [&] { return !seqs.empty() && seqs.back() == burst - 1; }));
  }
  client.stop();
  server.stop();
  std::lock_guard lock(m);
  return seqs;
}

} // namespace

// The request's max_queued_samples reaches the reactor: a burst far faster
// than the wire is received whole and in order when the queue is deep enough.
TEST(ClientServerContent, ImuBurstArrivesCompleteWithADeepQueue) {
  constexpr uint32_t kBurst = 400;
  const auto seqs = imuBurst("127.0.0.1:50077", kBurst, kBurst);
  ASSERT_EQ(seqs.size(), kBurst);
  for (uint32_t i = 0; i < kBurst; ++i) {
    ASSERT_EQ(seqs[i], i);
  }
}

TEST(SensorsServerTest, WaitBlocksUntilStop) {
  msensor::SensorsServer server({}, {.listen_address = "127.0.0.1:50072"});
  server.start();
  std::atomic<bool> returned{false};
  std::thread waiter([&] {
    server.wait();
    returned.store(true);
  });
  std::this_thread::sleep_for(100ms);
  EXPECT_FALSE(returned.load());
  server.stop();
  waiter.join();
  EXPECT_TRUE(returned.load());
}

TEST(SensorsServerTest, StartThrowsWhenTheAddressCannotBeBound) {
  msensor::SensorsServer server(
      {}, {.listen_address = "203.0.113.1:50073"}); // not a local address
  EXPECT_THROW(server.start(), std::runtime_error);
  server.stop(); // still safe
}

TEST(SensorsServerTest, LegacyPositionalConstructorServesLikeTheSensorsOne) {
  const std::string address = "127.0.0.1:50074";
  auto lidar = std::make_shared<msensor::SimLidar>(true);
  lidar->startSampling();
  msensor::SensorsServer server(nullptr, nullptr, nullptr, lidar, address);
  server.start();
  msensor::SensorsRemoteClient client(address);

  std::mutex m;
  std::condition_variable cv;
  bool got_scan = false;
  client.setScanCallback([&](const msensor::Scan3DI &) {
    std::lock_guard lock(m);
    got_scan = true;
    cv.notify_all();
  });
  client.start();
  {
    std::unique_lock lock(m);
    EXPECT_TRUE(cv.wait_for(lock, 5s, [&] { return got_scan; }));
  }
  client.stop();
  server.stop();
  lidar->stopSampling();
}

TEST(SensorsServerTest, RejectsJpegQualityOutsideZeroToHundred) {
  EXPECT_THROW(msensor::SensorsServer({}, {.camera_jpeg_quality = -1}),
               std::invalid_argument);
  EXPECT_THROW(msensor::SensorsServer({}, {.camera_jpeg_quality = 101}),
               std::invalid_argument);
  EXPECT_NO_THROW(msensor::SensorsServer({}, {.camera_jpeg_quality = 0}));
  EXPECT_NO_THROW(msensor::SensorsServer({}, {.camera_jpeg_quality = 100}));
}

namespace {

/// Serves a SimCamera at the given JPEG quality and returns the size of the
/// first frame's image_data received over a real stream.
size_t firstFrameBytes(const std::string &address, int quality) {
  auto camera = std::make_shared<msensor::SimCamera>();
  camera->startSampling();
  msensor::SensorsServer server(
      {.camera = camera},
      {.listen_address = address, .camera_jpeg_quality = quality});
  server.start();

  auto stub = sensors::CameraService::NewStub(
      grpc::CreateChannel(address, grpc::InsecureChannelCredentials()));
  grpc::ClientContext context;
  context.set_deadline(std::chrono::system_clock::now() + 10s);
  sensors::CameraStreamRequest request;
  auto reader = stub->getCameraFrame(&context, request);
  sensors::CameraStreamReply reply;
  size_t bytes = 0;
  if (reader->Read(&reply)) {
    bytes = reply.image_data().size();
  }
  context.TryCancel();
  reader->Finish();
  server.stop();
  camera->stopSampling();
  return bytes;
}

} // namespace

TEST(SensorsServerTest, CameraJpegQualityShrinksEncodedFrames) {
  const size_t low = firstFrameBytes("127.0.0.1:50075", 10);
  const size_t high = firstFrameBytes("127.0.0.1:50076", 95);
  EXPECT_GT(low, 0u);
  EXPECT_GT(high, 0u);
  EXPECT_LT(low, high);
}

namespace {

/// Pushes the sequence numbers one by one and waits for each to reach the
/// client, so the client's queue never decides what arrives.
void pushImuSequence(FeedSensor &feed, std::mutex &m,
                     std::condition_variable &cv,
                     const std::vector<uint32_t> &delivered,
                     std::initializer_list<uint32_t> numbers) {
  size_t expected;
  {
    std::lock_guard lock(m);
    expected = delivered.size();
  }
  for (const uint32_t seq : numbers) {
    msensor::IMUData imu{};
    imu.header = {1'000'000'000ULL + seq, seq};
    feed.push(imu);
    ++expected;
    std::unique_lock lock(m);
    ASSERT_TRUE(
        cv.wait_for(lock, 5s, [&] { return delivered.size() >= expected; }));
  }
}

} // namespace

// The client counts gaps in the sequence numbers of each stream and still
// delivers every sample, in order.
TEST(SensorsServerTest,
     ClientCountsSequenceGapsPerStreamAndDeliversEverything) {
  const std::string address = "127.0.0.1:50082";
  auto feed = std::make_shared<FeedSensor>();
  msensor::SensorsServer server({.imu = feed, .lidar = feed},
                                {.listen_address = address});
  server.start();
  msensor::SensorsRemoteClient mutable_client(
      address, {.scan_queue_size = 64, .imu_queue_size = 64});
  const msensor::SensorsRemoteClient &client = mutable_client; // const getters

  std::mutex m;
  std::condition_variable cv;
  std::vector<uint32_t> imus, scans;
  mutable_client.setImuCallback([&](const msensor::IMUData &imu) {
    std::lock_guard lock(m);
    imus.push_back(imu.header.sequence_number);
    cv.notify_all();
  });
  mutable_client.setScanCallback([&](const msensor::Scan3DI &scan) {
    std::lock_guard lock(m);
    scans.push_back(scan.header.sequence_number);
    cv.notify_all();
  });
  EXPECT_EQ(client.lostScans(), 0U);
  EXPECT_EQ(client.lostImuSamples(), 0U);
  mutable_client.start();
  for (int i = 0;
       i < 500 && !(feed->hasImuConsumer() && feed->hasScanConsumer()); ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(feed->hasImuConsumer());
  ASSERT_TRUE(feed->hasScanConsumer());

  // First number (100) means nothing; 100 -> 103 loses 2; 103 -> 104 nothing;
  // 104 -> 0xFFFFFFFE is too big a leap to tell from a restart; then the wrap
  // 0xFFFFFFFF -> 0 loses nothing and 0 -> 3 loses 2; 3 -> 1 is a restart;
  // 1 -> 5 loses 3.
  const std::vector<uint32_t> sent{100, 103, 104, 0xFFFFFFFEU, 0xFFFFFFFFU, 0,
                                   3,   1,   5};
  for (size_t i = 0; i < sent.size(); ++i) {
    msensor::IMUData imu{};
    imu.header = {1'000'000'000ULL + sent[i], sent[i]};
    feed->push(imu);
    std::unique_lock lock(m);
    ASSERT_TRUE(cv.wait_for(lock, 5s, [&] { return imus.size() > i; }));
  }
  for (const uint32_t seq :
       {7U, 8U, 12U}) { // the scan stream has its own count
    auto scan = makeScan(seq, 3);
    feed->push(scan);
    std::unique_lock lock(m);
    const size_t want = scans.size() + 1;
    ASSERT_TRUE(cv.wait_for(lock, 5s, [&] { return scans.size() >= want; }));
  }

  EXPECT_EQ(client.lostImuSamples(), 2U + 2U + 3U);
  EXPECT_EQ(client.lostScans(), 3U);
  {
    std::lock_guard lock(m);
    EXPECT_EQ(imus, sent); // counting changed nothing about delivery
    EXPECT_EQ(scans, (std::vector<uint32_t>{7, 8, 12}));
  }
  mutable_client.stop();
  server.stop();
}

// A reconnect starts a new stream: its first number is not compared with the
// last number of the previous connection.
TEST(SensorsServerTest, ReconnectStartsANewSequenceStream) {
  const std::string address = "127.0.0.1:50083";
  msensor::SensorsRemoteClient client(address, {.imu_queue_size = 64});
  std::mutex m;
  std::condition_variable cv;
  std::vector<uint32_t> imus;
  client.setImuCallback([&](const msensor::IMUData &imu) {
    std::lock_guard lock(m);
    imus.push_back(imu.header.sequence_number);
    cv.notify_all();
  });
  client.start();

  const auto session = [&](std::initializer_list<uint32_t> numbers) {
    auto feed = std::make_shared<FeedSensor>();
    msensor::SensorsServer server({.imu = feed}, {.listen_address = address});
    server.start();
    // The client reconnects after a second.
    for (int i = 0; i < 600 && !feed->hasImuConsumer(); ++i) {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(feed->hasImuConsumer());
    pushImuSequence(*feed, m, cv, imus, numbers);
    server.stop();
  };
  session({10, 11, 12});
  EXPECT_EQ(client.lostImuSamples(), 0U);
  // Without a reset, 12 -> 500 would count 487 samples.
  session({500, 501, 503});
  EXPECT_EQ(client.lostImuSamples(), 1U);
  client.stop();
  EXPECT_EQ(imus, (std::vector<uint32_t>{10, 11, 12, 500, 501, 503}));
}

// linkUp() tells a connected but silent server (a paused playback) from a dead
// one, and the sample ages tell silence from a stream that delivers.
TEST(SensorsServerTest, ClientReportsLinkAndSampleAges) {
  const std::string address = "127.0.0.1:50085";
  const auto wait_for = [](auto &&condition, std::chrono::milliseconds limit) {
    for (auto waited = 0ms; waited < limit; waited += 10ms) {
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return condition();
  };

  msensor::SensorsRemoteClient client(address, {.imu_queue_size = 64});
  EXPECT_EQ(client.address(), address);
  EXPECT_FALSE(client.linkUp());
  EXPECT_FALSE(client.lastScanAge().has_value());
  EXPECT_FALSE(client.lastImuAge().has_value());

  std::atomic<int> scans{0};
  std::atomic<int> imus{0};
  client.setScanCallback([&](const msensor::Scan3DI &) { ++scans; });
  client.setImuCallback([&](const msensor::IMUData &) { ++imus; });
  client.start();
  // No server yet: still down, no samples.
  std::this_thread::sleep_for(200ms);
  EXPECT_FALSE(client.linkUp());

  auto feed = std::make_shared<FeedSensor>();
  {
    msensor::SensorsServer server({.imu = feed, .lidar = feed},
                                  {.listen_address = address});
    server.start();

    // Connected, but the server sends nothing: link up, no ages.
    ASSERT_TRUE(wait_for([&] { return client.linkUp(); }, 6000ms));
    EXPECT_FALSE(client.lastScanAge().has_value());
    EXPECT_FALSE(client.lastImuAge().has_value());

    ASSERT_TRUE(wait_for(
        [&] { return feed->hasScanConsumer() && feed->hasImuConsumer(); },
        6000ms));
    feed->push(makeScan(0, 4));
    msensor::IMUData imu{};
    imu.header = {1'000'000'000ULL, 0};
    feed->push(imu);
    ASSERT_TRUE(wait_for([&] { return scans == 1 && imus == 1; }, 5000ms));

    ASSERT_TRUE(client.lastScanAge().has_value());
    ASSERT_TRUE(client.lastImuAge().has_value());
    EXPECT_LT(*client.lastScanAge(), 2000ms);
    EXPECT_LT(*client.lastImuAge(), 2000ms);

    // The ages grow while the server is silent.
    std::this_thread::sleep_for(300ms);
    EXPECT_GE(*client.lastScanAge(), 250ms);
    EXPECT_GE(*client.lastImuAge(), 250ms);
    EXPECT_TRUE(client.linkUp());
    server.stop();
  }

  // The server is gone: the link goes down; the last sample times stay.
  EXPECT_TRUE(wait_for([&] { return !client.linkUp(); }, 6000ms));
  EXPECT_TRUE(client.lastScanAge().has_value());
  client.stop();
}

// A recorder on the sensor host and a gRPC client read the same bus at once;
// the recording is complete and in order whatever the client does.
TEST(SensorsServerTest, RecordingWhileAClientStreamsFromTheSameBus) {
  const std::string address = "127.0.0.1:50084";
  constexpr uint32_t kImu = 600;
  constexpr uint32_t kScansEvery = 20;
  auto feed = std::make_shared<FeedSensor>();
  auto bus = std::make_shared<msensor::SensorBus>(
      msensor::Sensors{.imu = feed, .lidar = feed});
  const auto path = std::filesystem::temp_directory_path() /
                    ("msensor_record_while_streaming_" +
                     std::to_string(::getpid()) + ".pbscan");

  msensor::BusRecorder recorder(bus);
  recorder.start(path.string());
  msensor::SensorsServer server({.listen_address = address}, bus);
  server.start();

  msensor::SensorsRemoteClient client(
      address, {.scan_queue_size = 64, .imu_queue_size = 1000});
  std::mutex m;
  std::condition_variable cv;
  std::vector<uint32_t> client_imus;
  client.setImuCallback([&](const msensor::IMUData &imu) {
    std::lock_guard lock(m);
    client_imus.push_back(imu.header.sequence_number);
    cv.notify_all();
  });
  client.start();

  for (uint32_t i = 0; i < kImu; ++i) {
    msensor::IMUData imu{};
    imu.header = {1'000'000'000ULL + i, i};
    feed->push(imu);
    if (i % kScansEvery == 0) {
      feed->push(makeScan(i / kScansEvery, 50));
    }
    std::this_thread::sleep_for(500us);
  }
  {
    std::unique_lock lock(m);
    ASSERT_TRUE(cv.wait_for(lock, 10s, [&] {
      return !client_imus.empty() && client_imus.back() == kImu - 1;
    }));
  }
  client.stop();
  server.stop();
  const auto stats = recorder.stop();

  EXPECT_EQ(stats.imu.written, kImu);
  EXPECT_EQ(stats.lidar.written, kImu / kScansEvery);
  EXPECT_EQ(stats.imu.dropped + stats.lidar.dropped, 0U);

  size_t imus = 0, scans = 0;
  {
    msensor::ScanPlayer player(path);
    while (player.next()) {
      const auto &entry = player.getLastEntry();
      if (entry.has_imu()) {
        ASSERT_EQ(entry.imu().header().sequence_number(), imus);
        ++imus;
      } else if (entry.has_scan()) {
        ASSERT_EQ(entry.scan().header().sequence_number(), scans);
        ++scans;
      }
    }
  }
  EXPECT_EQ(imus, kImu);
  EXPECT_EQ(scans, kImu / kScansEvery);
  {
    // The client got an in-order, gap-free tail of the same samples.
    std::lock_guard lock(m);
    ASSERT_FALSE(client_imus.empty());
    for (size_t i = 1; i < client_imus.size(); ++i) {
      ASSERT_EQ(client_imus[i], client_imus[i - 1] + 1);
    }
    EXPECT_EQ(client.lostImuSamples(), 0U);
  }
  std::filesystem::remove(path);
}
