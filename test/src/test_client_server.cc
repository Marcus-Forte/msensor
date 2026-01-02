#include "sensors_remote_client.hh"
#include "sensors_server.hh"
#include <gtest/gtest.h>

class TestClientServer : public ::testing::Test {
public:
  void SetUp() override {
    server = std::make_shared<SensorsServer>();
    client = std::make_shared<SensorsRemoteClient>("localhost:50051");
  }

protected:
  std::shared_ptr<SensorsServer> server;
  std::shared_ptr<SensorsRemoteClient> client;
};

// TODO fix infinite loop
TEST_F(TestClientServer, DISABLED_TestServer) {

  server->start();

  // Wait for the data to reach the client.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  client->start();

  // Wait for client to open the streams
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto scan_read = client->getScan();
  auto imu_read = client->getImuData();
  // No data yet
  EXPECT_EQ(scan_read, nullptr);
  EXPECT_EQ(imu_read, std::nullopt);

  // Push data into the queues
  auto scan = std::make_shared<msensor::Scan3DI>();
  scan->points->emplace_back(1, 2, 3);
  scan->timestamp = 10;

  auto imu = msensor::IMUData(1, 2, 3, 4, 5, 6, 7);


  // Wait for data to reach the client
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  scan_read = client->getScan();
  imu_read = client->getImuData();
  ASSERT_NE(scan_read, nullptr);
  ASSERT_NE(imu_read, std::nullopt);

  const auto &scan_points = *scan_read->points;

  EXPECT_EQ(scan_read->timestamp, 10);
  ASSERT_GE(scan_read->points->size(), 1);
  EXPECT_EQ(scan_points[0].x, 1);
  EXPECT_EQ(scan_points[0].y, 2);
  EXPECT_EQ(scan_points[0].z, 3);

  EXPECT_EQ(imu_read->ax, 1);
  EXPECT_EQ(imu_read->ay, 2);
  EXPECT_EQ(imu_read->az, 3);
  EXPECT_EQ(imu_read->gx, 4);
  EXPECT_EQ(imu_read->gy, 5);
  EXPECT_EQ(imu_read->gz, 6);

  std::cout << "Test complete, stopping server and client." << std::endl;

  client->stop();
  server->stop();
}