#include "lidar.grpc.pb.h"
#include "msensor/imu/sim_imu.hh"
#include "msensor/lidar/sim_lidar.hh"
#include "msensor_server.hh"
#include "sensors_remote_client.hh"
#include <condition_variable>
#include <grpcpp/grpcpp.h>
#include <gtest/gtest.h>
#include <mutex>

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
