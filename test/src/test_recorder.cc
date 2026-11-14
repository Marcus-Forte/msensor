#include "IFileMock.hh"
#include "msensor/conversions/lidar_imu_conversions.hh"
#include "msensor/file/file.hh"
#include "msensor/recorder/scan_player.hh"
#include "msensor/recorder/scan_recorder.hh"
#include "recording.pb.h"
#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace testing;
using namespace msensor;

class TestRecorder : public ::testing::Test {
public:
  void SetUp() override {
    file_mock_ = std::make_shared<testing::StrictMock<IFileMock>>();
    recorder_ = std::make_shared<ScanRecorder>(file_mock_);

    EXPECT_CALL(*file_mock_, close()); // from destructor
  }

protected:
  std::shared_ptr<ScanRecorder> recorder_;
  std::shared_ptr<testing::StrictMock<IFileMock>> file_mock_;
};

TEST_F(TestRecorder, start) {
  EXPECT_CALL(*file_mock_, open(testing::_));

  recorder_->start();
}

namespace {
/// Collects what a recorder writes: each write() call and the concatenation.
struct Captured {
  std::vector<std::string> writes;
  std::string all;
  int flushes = 0;
};

void captureWrites(IFileMock &mock, Captured &out) {
  EXPECT_CALL(mock, write(_, _))
      .WillRepeatedly([&out](const char *data, size_t size) {
        out.writes.emplace_back(data, size);
        out.all.append(data, size);
      });
  EXPECT_CALL(mock, flush()).WillRepeatedly([&out] { ++out.flushes; });
}

/// The entry is a native-endian uint64 byte count, then that many bytes.
void expectFramedEntry(const Captured &c, size_t payload_size) {
  ASSERT_EQ(c.writes.size(), 2U);
  ASSERT_EQ(c.writes[0].size(), sizeof(uint64_t));
  uint64_t prefix;
  std::memcpy(&prefix, c.writes[0].data(), sizeof(prefix));
  EXPECT_EQ(prefix, payload_size);
  EXPECT_EQ(c.writes[1].size(), payload_size);
  EXPECT_EQ(c.flushes, 1);
}
} // namespace

TEST_F(TestRecorder, record_scan) {
  EXPECT_CALL(*file_mock_, open(testing::_));
  Captured captured;
  captureWrites(*file_mock_, captured);
  auto scan = Scan3DI{};
  // Add points. Zeros may not be serialized.
  scan.points.emplace_back(1, 2, 3);
  scan.points.emplace_back(1, 2, 3);
  scan.header.timestamp = 10;

  recorder_->start();
  recorder_->record(scan);

  expectFramedEntry(captured, 40);
  sensors::RecordingEntry entry;
  ASSERT_TRUE(entry.ParseFromString(captured.writes[1]));
  ASSERT_TRUE(entry.has_scan());
  EXPECT_EQ(entry.scan().header().timestamp(), 10U);
}

TEST_F(TestRecorder, record_imu) {
  EXPECT_CALL(*file_mock_, open(testing::_));
  Captured captured;
  captureWrites(*file_mock_, captured);
  const auto imu = IMUData{Header{10, 0},
                           1,
                           2,
                           3,
                           4,
                           5,
                           6}; // Add values. Zeros may not be serialized.

  recorder_->start();
  recorder_->record(imu);

  expectFramedEntry(captured, 36); // 1 imu
  sensors::RecordingEntry entry;
  ASSERT_TRUE(entry.ParseFromString(captured.writes[1]));
  ASSERT_TRUE(entry.has_imu());
}

TEST_F(TestRecorder, write_failure_surfaces_from_record) {
  EXPECT_CALL(*file_mock_, open(testing::_));
  EXPECT_CALL(*file_mock_, write(_, _))
      .WillOnce(Throw(std::runtime_error("disk full")));
  recorder_->start();
  EXPECT_THROW(recorder_->record(IMUData{Header{10, 0}, 1, 2, 3, 4, 5, 6}),
               std::runtime_error);
}

TEST_F(TestRecorder, flush_failure_surfaces_from_record) {
  EXPECT_CALL(*file_mock_, open(testing::_));
  EXPECT_CALL(*file_mock_, write(_, _)).Times(2);
  EXPECT_CALL(*file_mock_, flush())
      .WillOnce(Throw(std::runtime_error("disk full")));
  recorder_->start();
  EXPECT_THROW(recorder_->record(IMUData{Header{10, 0}, 1, 2, 3, 4, 5, 6}),
               std::runtime_error);
}
TEST_F(TestRecorder, records_nothing_before_start_or_after_stop) {
  // StrictMock: any write()/flush() call would fail the test.
  recorder_->record(Scan3DI{});
  recorder_->record(IMUData{});

  EXPECT_CALL(*file_mock_, open(testing::_));
  // From stop(); the fixture expects the destructor's close().
  EXPECT_CALL(*file_mock_, close()).RetiresOnSaturation();
  recorder_->start("unused.pbscan");
  recorder_->stop();
  recorder_->record(Scan3DI{});
  recorder_->record(IMUData{});
}

// --- Recordings on disk: ScanRecorder + File written, ScanPlayer read. ---

namespace {

class ScratchDir {
public:
  ScratchDir() {
    path_ = std::filesystem::temp_directory_path() /
            ("msensor_test_" + std::to_string(::getpid()) + "_" +
             ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::remove_all(path_);
    std::filesystem::create_directories(path_);
  }
  ~ScratchDir() { std::filesystem::remove_all(path_); }
  std::filesystem::path operator/(const std::string &name) const {
    return path_ / name;
  }

private:
  std::filesystem::path path_;
};

Scan3DI makeScan(uint32_t seq, size_t points) {
  Scan3DI scan;
  scan.header = Header{1'000'000ULL * seq + 7, seq};
  for (size_t i = 0; i < points; ++i) {
    const float f = static_cast<float>(seq * 1000 + i);
    scan.points.push_back(
        {f * 0.5F, -f, f * 0.25F, static_cast<float>(i % 256)});
  }
  return scan;
}

IMUData makeImu(uint32_t seq) {
  const float f = static_cast<float>(seq);
  return IMUData{
      Header{1'000'000ULL * seq + 3, seq}, f, -f, 9.81F, f / 2, 0.0F, -f / 4};
}

void expectSameScan(const Scan3DI &a, const Scan3DI &b) {
  EXPECT_EQ(a.header.timestamp, b.header.timestamp);
  EXPECT_EQ(a.header.sequence_number, b.header.sequence_number);
  ASSERT_EQ(a.points.size(), b.points.size());
  if (!a.points.empty()) { // memcmp(nullptr, ...) is undefined
    EXPECT_EQ(std::memcmp(a.points.data(), b.points.data(),
                          a.points.size() * sizeof(Point3I)),
              0);
  }
}

void expectSameImu(const IMUData &a, const IMUData &b) {
  EXPECT_EQ(a.header.timestamp, b.header.timestamp);
  EXPECT_EQ(a.header.sequence_number, b.header.sequence_number);
  EXPECT_EQ(std::memcmp(&a.ax, &b.ax, 6 * sizeof(float)), 0);
}

size_t openFileDescriptors() {
  size_t count = 0;
  for ([[maybe_unused]] const auto &entry :
       std::filesystem::directory_iterator("/proc/self/fd")) {
    ++count;
  }
  return count;
}

bool isMapped(const std::filesystem::path &file) {
  std::ifstream maps("/proc/self/maps");
  std::string line;
  while (std::getline(maps, line)) {
    if (line.find(file.string()) != std::string::npos) {
      return true;
    }
  }
  return false;
}

} // namespace

TEST(FileTest, WriteHonoursSize) {
  ScratchDir dir;
  const auto path = dir / "bytes.bin";
  {
    File file;
    file.open(path.string());
    file.write("abcdefghijk", 3);
    file.write("XY", 2);
    file.close();
  }
  std::ifstream in(path, std::ios::binary);
  const std::string content((std::istreambuf_iterator<char>(in)), {});
  EXPECT_EQ(content, "abcXY");
}

TEST(FileTest, WriteAndFlushToFullDeviceThrow) {
  if (!std::filesystem::exists("/dev/full")) {
    GTEST_SKIP() << "/dev/full not available";
  }
  File file;
  file.open("/dev/full");
  // Buffered: the failure may show on write() or only on flush().
  EXPECT_THROW(
      {
        file.write("x", 1);
        file.flush();
      },
      std::runtime_error);
  // The stream stays failed.
  EXPECT_THROW(file.write("x", 1), std::runtime_error);
  EXPECT_THROW(file.flush(), std::runtime_error);
}

TEST(FileTest, OpenFailureThrows) {
  File file;
  EXPECT_THROW(file.open("/nonexistent-dir/x/y.pbscan"), std::runtime_error);
}

TEST(RecordingRoundTrip, ScansAndImuComeBackInOrder) {
  ScratchDir dir;
  const auto path = dir / "round_trip.pbscan";
  std::vector<Scan3DI> scans;
  std::vector<IMUData> imus;
  {
    ScanRecorder recorder(std::make_shared<File>());
    recorder.start(path.string());
    EXPECT_EQ(recorder.getFilename(), path.string());
    // Interleave entry kinds and sizes, including empty scans, so the reused
    // messages and buffer are exercised in every transition.
    for (uint32_t i = 0; i < 30; ++i) {
      if (i % 3 != 1) {
        scans.push_back(makeScan(i, (i * 37) % 2000));
        recorder.record(scans.back());
      }
      imus.push_back(makeImu(i));
      recorder.record(imus.back());
    }
    recorder.stop();
  }

  ScanPlayer player(path);
  size_t scan_index = 0;
  size_t imu_index = 0;
  for (uint32_t i = 0; i < 30; ++i) {
    if (i % 3 != 1) {
      ASSERT_TRUE(player.next());
      ASSERT_TRUE(player.getLastEntry().has_scan());
      expectSameScan(fromProtobuf(player.getLastEntry().scan()),
                     scans[scan_index++]);
    }
    ASSERT_TRUE(player.next());
    ASSERT_TRUE(player.getLastEntry().has_imu());
    expectSameImu(fromProtobuf(player.getLastEntry().imu()), imus[imu_index++]);
  }
  EXPECT_FALSE(player.next());

  // reset() replays from the first entry.
  player.reset();
  ASSERT_TRUE(player.next());
  expectSameScan(fromProtobuf(player.getLastEntry().scan()), scans.front());
}

TEST(RecordingRoundTrip, EntryFramingIsSizePrefixedProtobuf) {
  ScratchDir dir;
  const auto path = dir / "framing.pbscan";
  const auto imu = makeImu(5);
  {
    ScanRecorder recorder(std::make_shared<File>());
    recorder.start(path.string());
    recorder.record(imu);
  }
  std::ifstream in(path, std::ios::binary);
  const std::string content((std::istreambuf_iterator<char>(in)), {});
  ASSERT_GE(content.size(), sizeof(uint64_t));
  uint64_t size;
  std::memcpy(&size, content.data(), sizeof(size));
  ASSERT_EQ(size, content.size() - sizeof(uint64_t));
  sensors::RecordingEntry entry;
  ASSERT_TRUE(entry.ParseFromString(content.substr(sizeof(uint64_t))));
  ASSERT_TRUE(entry.has_imu());
  expectSameImu(fromProtobuf(entry.imu()), imu);
}

// The LiDAR and IMU reader threads of remote_recorder record concurrently.
TEST(RecordingRoundTrip, ConcurrentRecordersInterleaveWholeEntries) {
  ScratchDir dir;
  const auto path = dir / "concurrent.pbscan";
  constexpr uint32_t kScans = 60;
  constexpr uint32_t kImus = 600;
  {
    ScanRecorder recorder(std::make_shared<File>());
    recorder.start(path.string());
    std::thread lidar([&] {
      for (uint32_t i = 0; i < kScans; ++i) {
        recorder.record(makeScan(i, 500 + i));
      }
    });
    std::thread imu([&] {
      for (uint32_t i = 0; i < kImus; ++i) {
        recorder.record(makeImu(i));
      }
    });
    lidar.join();
    imu.join();
    recorder.stop();
  }

  ScanPlayer player(path);
  uint32_t next_scan = 0;
  uint32_t next_imu = 0;
  while (player.next()) {
    const auto &entry = player.getLastEntry();
    if (entry.has_scan()) {
      expectSameScan(fromProtobuf(entry.scan()),
                     makeScan(next_scan, 500 + next_scan));
      ++next_scan;
    } else {
      ASSERT_TRUE(entry.has_imu());
      expectSameImu(fromProtobuf(entry.imu()), makeImu(next_imu));
      ++next_imu;
    }
  }
  EXPECT_EQ(next_scan, kScans);
  EXPECT_EQ(next_imu, kImus);
}

TEST(ScanPlayerTest, EmptyRecordingHasNoEntries) {
  ScratchDir dir;
  const auto path = dir / "empty.pbscan";
  std::ofstream(path).close();
  ScanPlayer player(path);
  EXPECT_FALSE(player.next());
}

TEST(ScanPlayerTest, MissingFileThrows) {
  EXPECT_THROW(ScanPlayer("/nonexistent/recording.pbscan"), std::runtime_error);
}

TEST(ScanPlayerTest, TruncatedEntryThrows) {
  ScratchDir dir;
  const auto path = dir / "truncated.pbscan";
  {
    ScanRecorder recorder(std::make_shared<File>());
    recorder.start(path.string());
    recorder.record(makeScan(1, 100));
  }
  std::filesystem::resize_file(path, std::filesystem::file_size(path) - 10);
  ScanPlayer player(path);
  EXPECT_THROW(player.next(), std::runtime_error);

  std::filesystem::resize_file(path, 5); // not even a whole size prefix
  ScanPlayer short_player(path);
  EXPECT_THROW(short_player.next(), std::runtime_error);
}

TEST(ScanPlayerTest, ReleasesMappingAndDescriptor) {
  ScratchDir dir;
  const auto path = dir / "lifetime.pbscan";
  {
    ScanRecorder recorder(std::make_shared<File>());
    recorder.start(path.string());
    recorder.record(makeScan(1, 1000));
  }
  const size_t fds_before = openFileDescriptors();
  for (int i = 0; i < 20; ++i) {
    ScanPlayer player(path);
    ASSERT_TRUE(player.next());
    EXPECT_TRUE(isMapped(path));
  }
  EXPECT_EQ(openFileDescriptors(), fds_before);
  EXPECT_FALSE(isMapped(path));
}
