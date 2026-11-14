#include "msensor/conversions/conversions.hh"

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <thread>
#include <vector>

namespace {

msensor::Scan3DI makeScan(size_t points, uint32_t seed) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> coord(-50.0F, 50.0F);
  std::uniform_int_distribution<int> reflectivity(0, 255);
  msensor::Scan3DI scan;
  scan.header.timestamp = 1'700'000'000'000'000'000ULL + seed;
  scan.header.sequence_number = seed;
  scan.points.resize(points);
  for (auto &p : scan.points) {
    p.x = coord(gen);
    p.y = coord(gen);
    p.z = coord(gen);
    p.intensity = static_cast<float>(reflectivity(gen));
  }
  return scan;
}

/// Bitwise equality, so -0.0 vs 0.0 or NaN payloads would be caught too.
void expectSameScan(const msensor::Scan3DI &expected,
                    const msensor::Scan3DI &actual) {
  EXPECT_EQ(actual.header.timestamp, expected.header.timestamp);
  EXPECT_EQ(actual.header.sequence_number, expected.header.sequence_number);
  ASSERT_EQ(actual.points.size(), expected.points.size());
  if (!expected.points.empty()) { // memcmp(nullptr, ...) is undefined
    EXPECT_EQ(std::memcmp(actual.points.data(), expected.points.data(),
                          expected.points.size() * sizeof(msensor::Point3I)),
              0);
  }
}

void expectColumns(const msensor::Scan3DI &scan,
                   const sensors::PointCloud3 &msg) {
  EXPECT_EQ(msg.header().timestamp(), scan.header.timestamp);
  EXPECT_EQ(msg.header().sequence_number(), scan.header.sequence_number);
  ASSERT_EQ(msg.x_size(), static_cast<int>(scan.points.size()));
  ASSERT_EQ(msg.y_size(), msg.x_size());
  ASSERT_EQ(msg.z_size(), msg.x_size());
  ASSERT_EQ(msg.intensity_size(), msg.x_size());
  EXPECT_EQ(msg.r_size(), 0);
  EXPECT_EQ(msg.g_size(), 0);
  EXPECT_EQ(msg.b_size(), 0);
  for (int i = 0; i < msg.x_size(); ++i) {
    const auto &p = scan.points[static_cast<size_t>(i)];
    ASSERT_EQ(msg.x(i), p.x) << "point " << i;
    ASSERT_EQ(msg.y(i), p.y) << "point " << i;
    ASSERT_EQ(msg.z(i), p.z) << "point " << i;
    ASSERT_EQ(msg.intensity(i), static_cast<uint32_t>(p.intensity))
        << "point " << i;
  }
}

} // namespace

// Sizes cover the empty scan, the scalar tail of the vectorised loop (sizes
// that are not a multiple of the vector width), one Mid360 scan (100 packets
// of 96 points) and a dense cloud.
class ScanConversion : public ::testing::TestWithParam<size_t> {};

TEST_P(ScanConversion, ToProtobufFillsEveryColumn) {
  const auto scan = makeScan(GetParam(), 7);
  sensors::PointCloud3 msg;
  toProtobuf(scan, msg);
  expectColumns(scan, msg);
}

TEST_P(ScanConversion, RoundTripIsBitExact) {
  const auto scan = makeScan(GetParam(), 11);
  sensors::PointCloud3 msg;
  toProtobuf(scan, msg);
  expectSameScan(scan, fromProtobuf(msg));
}

TEST_P(ScanConversion, RoundTripSurvivesSerialization) {
  const auto scan = makeScan(GetParam(), 13);
  sensors::PointCloud3 msg;
  toProtobuf(scan, msg);
  sensors::PointCloud3 parsed;
  ASSERT_TRUE(parsed.ParseFromString(msg.SerializeAsString()));
  expectSameScan(scan, fromProtobuf(parsed));
}

INSTANTIATE_TEST_SUITE_P(Sizes, ScanConversion,
                         ::testing::Values(0, 1, 2, 3, 4, 5, 7, 8, 15, 17, 2000,
                                           9600, 100003));

TEST(ScanConversionReuse, ReusedMessageHoldsOnlyTheLatestScan) {
  sensors::PointCloud3 msg;
  const auto big = makeScan(9600, 1);
  const auto small = makeScan(5, 2);
  const auto empty = makeScan(0, 3);

  toProtobuf(big, msg);
  expectColumns(big, msg);
  toProtobuf(small, msg);
  expectColumns(small, msg);
  toProtobuf(empty, msg);
  expectColumns(empty, msg);
  toProtobuf(big, msg);
  expectColumns(big, msg);
}

TEST(ScanConversionReuse, StaleColorAndHeaderFieldsAreCleared) {
  sensors::PointCloud3 msg;
  msg.add_r(1.0F);
  msg.add_g(2.0F);
  msg.add_b(3.0F);
  msg.mutable_header()->set_timestamp(99);
  const auto scan = makeScan(3, 4);
  toProtobuf(scan, msg);
  expectColumns(scan, msg);
}

TEST(ScanConversionValues, IntensityIsTruncatedToAnInteger) {
  msensor::Scan3DI scan;
  scan.points.push_back({1.5F, -2.25F, 0.0F, 7.9F});
  scan.points.push_back({-0.0F, 3.0F, 1e-30F, 255.0F});
  sensors::PointCloud3 msg;
  toProtobuf(scan, msg);
  ASSERT_EQ(msg.intensity_size(), 2);
  EXPECT_EQ(msg.intensity(0), 7U);
  EXPECT_EQ(msg.intensity(1), 255U);
  EXPECT_EQ(msg.x(0), 1.5F);
  EXPECT_EQ(msg.y(0), -2.25F);
  EXPECT_TRUE(std::signbit(msg.x(1)));
  EXPECT_EQ(msg.z(1), 1e-30F);
}

TEST(ScanConversionValues, MismatchedColumnsGiveAnEmptyScan) {
  sensors::PointCloud3 msg;
  msg.add_x(1.0F);
  msg.add_y(2.0F);
  msg.add_z(3.0F); // no intensity
  EXPECT_TRUE(fromProtobuf(msg).points.empty());
}

TEST(ImuConversion, RoundTripIsExact) {
  const msensor::IMUData imu{msensor::Header{123456789012345ULL, 42},
                             0.5F,
                             -9.81F,
                             1e-7F,
                             -0.0F,
                             3.25F,
                             -1.0F};
  sensors::IMUData msg;
  msg.set_ax(100.0F); // stale value must not survive
  toProtobuf(imu, msg);
  const auto back = fromProtobuf(msg);
  EXPECT_EQ(back.header.timestamp, imu.header.timestamp);
  EXPECT_EQ(back.header.sequence_number, imu.header.sequence_number);
  EXPECT_EQ(std::memcmp(&back.ax, &imu.ax, 6 * sizeof(float)), 0);
}

namespace {
msensor::CameraFrame makeFrame(int width, int height, uint32_t seq) {
  msensor::CameraFrame frame;
  frame.header = {1000U + seq, seq};
  frame.mat.create(height, width, CV_8UC3);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      frame.mat.at<cv::Vec3b>(y, x) =
          cv::Vec3b(static_cast<uchar>(x * 4), static_cast<uchar>(y * 4),
                    static_cast<uchar>(seq));
    }
  }
  return frame;
}
} // namespace

TEST(CameraConversion, EncodesJpegWithHeaderAndSize) {
  const auto frame = makeFrame(64, 48, 3);
  sensors::CameraStreamReply reply;
  toProtobuf(frame, reply);
  EXPECT_EQ(reply.encoding(), sensors::CameraEncoding::MJPEG);
  EXPECT_EQ(reply.width(), 64U);
  EXPECT_EQ(reply.height(), 48U);
  EXPECT_EQ(reply.header().timestamp(), frame.header.timestamp);
  EXPECT_EQ(reply.header().sequence_number(), 3U);
  const std::vector<uchar> bytes(reply.image_data().begin(),
                                 reply.image_data().end());
  const cv::Mat decoded = cv::imdecode(bytes, cv::IMREAD_COLOR);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.cols, 64);
  EXPECT_EQ(decoded.rows, 48);
}

TEST(CameraConversion, EmptyFrameSendsNoStaleImage) {
  sensors::CameraStreamReply reply;
  toProtobuf(makeFrame(32, 16, 1), reply);
  ASSERT_FALSE(reply.image_data().empty());

  msensor::CameraFrame empty;
  empty.header = {5, 9};
  toProtobuf(empty, reply);
  EXPECT_EQ(reply.encoding(), sensors::CameraEncoding::UNKNOWN);
  EXPECT_TRUE(reply.image_data().empty());
  EXPECT_EQ(reply.header().sequence_number(), 9U);
}

// Frames encoded concurrently must not share an encode buffer.
TEST(CameraConversion, ConcurrentEncodesKeepTheirOwnImages) {
  constexpr int kThreads = 4;
  constexpr int kFrames = 20;
  std::vector<std::thread> threads;
  std::vector<int> failures(kThreads, 0);
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([t, &failures] {
      // Different sizes per thread, so a shared buffer would mix them up.
      const auto frame =
          makeFrame(16 * (t + 1), 8 * (t + 1), static_cast<uint32_t>(t));
      sensors::CameraStreamReply reply;
      for (int i = 0; i < kFrames; ++i) {
        toProtobuf(frame, reply);
        const std::vector<uchar> bytes(reply.image_data().begin(),
                                       reply.image_data().end());
        const cv::Mat decoded = cv::imdecode(bytes, cv::IMREAD_COLOR);
        if (decoded.cols != frame.mat.cols || decoded.rows != frame.mat.rows) {
          ++failures[static_cast<size_t>(t)];
        }
      }
    });
  }
  for (auto &thread : threads) {
    thread.join();
  }
  for (int t = 0; t < kThreads; ++t) {
    EXPECT_EQ(failures[static_cast<size_t>(t)], 0) << "thread " << t;
  }
}
