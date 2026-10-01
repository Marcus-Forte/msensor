#include "msensor/imu/sim_imu.hh"
#include "msensor/lidar/sim_lidar.hh"
#include "msensor_server.hh"
#include "sensors_remote_client.hh"
#include <grpcpp/grpcpp.h>
#include <gtest/gtest.h>

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
  auto scan_sub = client->scans().subscribe();
  auto imu_sub = client->imu().subscribe();

  server->start();
  lidar->startSampling();
  imu->startSampling();
  client->start();

  const auto scan = scan_sub->waitPop({}, 5s);
  const auto imu_sample = imu_sub->waitPop({}, 5s);
  ASSERT_NE(scan, nullptr);
  ASSERT_NE(imu_sample, nullptr);
  EXPECT_EQ(scan->points->size(), 2000u);
  EXPECT_GT(scan->header.timestamp, 0u);
}

TEST_F(TestClientServer, TwoClientsBothReceiveScans) {
  SensorsRemoteClient second("localhost:50051");
  auto sub_a = client->scans().subscribe();
  auto sub_b = second.scans().subscribe();

  server->start();
  lidar->startSampling();
  client->start();
  second.start();

  EXPECT_NE(sub_a->waitPop({}, 5s), nullptr);
  EXPECT_NE(sub_b->waitPop({}, 5s), nullptr);
  second.stop();
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

TEST_F(TestClientServer, SubSampledStreamFiltersAndSurvivesCancel) {
  server->start();
  lidar->startSampling();

  auto channel = grpc::CreateChannel("localhost:50051",
                                     grpc::InsecureChannelCredentials());
  auto stub = sensors::LidarService::NewStub(channel);

  for (int round = 0; round < 3; ++round) {
    grpc::ClientContext context;
    auto stream = stub->getSubSampledLidarScan(&context);

    sensors::SubSampledLidarStreamRequest request;
    request.set_voxel_size(5.0f);
    ASSERT_TRUE(stream->Write(request));

    // Invalid sizes are ignored, the stream keeps working.
    request.set_voxel_size(-1.0f);
    ASSERT_TRUE(stream->Write(request));

    sensors::PointCloud3 cloud;
    ASSERT_TRUE(stream->Read(&cloud));
    // 2000 random points in a 20 m cube with 5 m voxels: at most 4^3.
    EXPECT_GT(cloud.x_size(), 0);
    EXPECT_LE(cloud.x_size(), 64);

    // Drop the stream mid-flight; the server must clean up.
    context.TryCancel();
  }
}
