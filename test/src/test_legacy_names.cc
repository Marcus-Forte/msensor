// Compiled WITHOUT MSENSOR_NO_LEGACY_NAMES: proves the old global spellings
// still compile for downstream code. It never starts a server.
#include <gtest/gtest.h>

#include <memory>
#include <type_traits>

#include "msensor/conversions/conversions.hh"
#include "msensor/lidar/sim_lidar.hh"
#include "msensor/timing/timing.hh"
#include "msensor_server.hh"
#include "sensors_remote_client.hh"

#ifdef MSENSOR_NO_LEGACY_NAMES
#error "test_legacy_names must be built without MSENSOR_NO_LEGACY_NAMES"
#endif

TEST(LegacyNames, ClassesAreTheNamespacedOnes) {
  static_assert(std::is_same_v<SensorsServer, msensor::SensorsServer>);
  static_assert(
      std::is_same_v<SensorsRemoteClient, msensor::SensorsRemoteClient>);
  static_assert(std::is_same_v<LidarServiceImpl, msensor::LidarServiceImpl>);
  static_assert(std::is_same_v<ImuServiceImpl, msensor::ImuServiceImpl>);
  static_assert(std::is_same_v<CameraServiceImpl, msensor::CameraServiceImpl>);
  static_assert(std::is_same_v<AdcServiceImpl, msensor::AdcServiceImpl>);
}

TEST(LegacyNames, ServerWithPositionalConstructorIsConstructible) {
  auto lidar = std::make_shared<msensor::SimLidar>(true);
  SensorsServer server(nullptr, nullptr, nullptr, lidar); // not started
  SensorsServer defaults;
  (void)server;
  (void)defaults;
}

TEST(LegacyNames, ServicesAreConstructible) {
  LidarServiceImpl lidar(nullptr);
  ImuServiceImpl imu(nullptr);
  CameraServiceImpl camera(nullptr);
  AdcServiceImpl adc(nullptr);
  (void)lidar;
  (void)imu;
  (void)camera;
  (void)adc;
}

TEST(LegacyNames, ClientTypeNamesTheInterfaces) {
  static_assert(std::is_base_of_v<msensor::ILidar, SensorsRemoteClient>);
  static_assert(std::is_base_of_v<msensor::IImu, SensorsRemoteClient>);
}

TEST(LegacyNames, UnqualifiedConversionsRoundTrip) {
  msensor::Scan3DI scan;
  scan.header = {123, 4};
  scan.points.resize(2);
  scan.points[1].x = 1.5f;
  sensors::PointCloud3 cloud;
  toProtobuf(scan, cloud);
  const msensor::Scan3DI back = fromProtobuf(cloud);
  ASSERT_EQ(back.points.size(), 2u);
  EXPECT_EQ(back.points[1].x, 1.5f);
  EXPECT_EQ(back.header.timestamp, 123u);

  msensor::IMUData imu;
  imu.header = {9, 1};
  imu.ax = 2.0f;
  sensors::IMUData msg;
  toProtobuf(imu, msg);
  EXPECT_EQ(fromProtobuf(msg).ax, 2.0f);
  EXPECT_EQ(fromProtobuf(msg).header.timestamp, 9u);
}

TEST(LegacyNames, GlobalTimingNamespace) {
  EXPECT_GT(timing::getNowNs(), 0u);
  EXPECT_GT(timing::getNowUs(), 0u);
  EXPECT_GT(timing::getEpochTimeNs(), 0u);
  EXPECT_GT(timing::getEpochTimeUs(), 0u);
}
