#include "lidar.grpc.pb.h"
#include "msensor/imu/sim_imu.hh"
#include "msensor/interface/CallbackSlot.hh"
#include "msensor/lidar/sim_lidar.hh"
#include "msensor_server.hh"
#include "sensors_remote_client.hh"
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <grpcpp/grpcpp.h>
#include <gtest/gtest.h>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class TestClientServer : public ::testing::Test {
public:
  void SetUp() override {
    lidar = std::make_shared<msensor::SimLidar>(true);
    imu = std::make_shared<msensor::SimImu>();
    server = std::make_shared<SensorsServer>(nullptr, nullptr, imu, lidar);
    client = std::make_shared<SensorsRemoteClient>("localhost:50051");
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
  std::shared_ptr<SensorsServer> server;
  std::shared_ptr<SensorsRemoteClient> client;
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

TEST_F(TestClientServer, SecondClientIsRejected) {
  server->start();
  lidar->startSampling();

  auto channel = grpc::CreateChannel("localhost:50051",
                                     grpc::InsecureChannelCredentials());
  auto stub = sensors::LidarService::NewStub(channel);
  const sensors::LidarStreamRequest request;

  grpc::ClientContext first_context;
  auto first = stub->getLidarScan(&first_context, request);
  sensors::PointCloud3 msg;
  ASSERT_TRUE(first->Read(&msg));

  grpc::ClientContext second_context;
  auto second = stub->getLidarScan(&second_context, request);
  sensors::PointCloud3 rejected;
  EXPECT_FALSE(second->Read(&rejected));
  EXPECT_EQ(second->Finish().error_code(),
            grpc::StatusCode::RESOURCE_EXHAUSTED);

  first_context.TryCancel();
  first->Finish();
}

TEST_F(TestClientServer, MissingSensorsReportUnavailableWithoutCrashing) {
  auto empty_server = std::make_shared<SensorsServer>();
  empty_server->start();
  SensorsRemoteClient c("localhost:50051");
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
  void push(msensor::Scan3DI scan) { scans_.emit(std::move(scan)); }

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
  SensorsServer server(nullptr, nullptr, nullptr, feed, address);
  server.start();
  SensorsRemoteClient client(address);

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

TEST(SensorsServerTest, WaitBlocksUntilStop) {
  SensorsServer server(nullptr, nullptr, nullptr, nullptr, "127.0.0.1:50072");
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
  SensorsServer server(nullptr, nullptr, nullptr, nullptr,
                       "203.0.113.1:50073"); // not a local address
  EXPECT_THROW(server.start(), std::runtime_error);
  server.stop(); // still safe
}
