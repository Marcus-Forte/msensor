// Benchmark for the msensor data paths that downstream SLAM exercises:
// msensor <-> protobuf conversions, recording playback and recording, and
// LiDAR/IMU delivery through the gRPC server and remote client.
//
// Built with the tests but not registered with CTest. Every row prints the
// best time over the in-process passes and a checksum of the row's output, so
// two builds can be compared for equivalence as well as speed:
//
//   bench_msensor [--passes N] [--grpc-scans N] [--grpc-imus N]
//                 [--tmp DIR] [--only SUBSTR] [recording.pbscan ...]
//
// Output rows have the form
//   ROW <name> <best_ms> <per_op_us> <ops> <checksum>
// Rows whose checksum differs between passes print "UNSTABLE".

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "msensor/conversions/conversions.hh"
#include "msensor/file/file.hh"
#include "msensor/interface/CallbackSlot.hh"
#include "msensor/recorder/recording_driver.hh"
#include "msensor/recorder/scan_player.hh"
#include "msensor/recorder/scan_recorder.hh"
#include "msensor_server.hh"
#include "recording.pb.h"
#include <opencv2/imgcodecs.hpp>
#include "sensors_remote_client.hh"

namespace {

using Clock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

// ---------------------------------------------------------------------------
// Checksums: word-wise multiply/xor hash over the exact bit patterns, so a
// single differing bit (including -0.0 vs 0.0) changes the result.

struct Hasher {
  uint64_t h = 0x243F6A8885A308D3ULL;

  void word(uint64_t w) {
    h ^= w;
    h *= 0x9E3779B97F4A7C15ULL;
    h ^= h >> 29;
  }

  void bytes(const void *data, size_t n) {
    const auto *c = static_cast<const unsigned char *>(data);
    word(n);
    size_t i = 0;
    for (; i + 8 <= n; i += 8) {
      uint64_t w;
      std::memcpy(&w, c + i, 8);
      word(w);
    }
    uint64_t tail = 0;
    if (n > i) { // c may be null when n == 0
      std::memcpy(&tail, c + i, n - i);
    }
    word(tail);
  }

  void f32(float v) {
    uint32_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    word(bits);
  }
};

void hashScan(Hasher &hs, const msensor::Scan3DI &scan) {
  hs.word(scan.header.timestamp);
  hs.word(scan.header.sequence_number);
  hs.bytes(scan.points.data(), scan.points.size() * sizeof(msensor::Point3I));
}

void hashImu(Hasher &hs, const msensor::IMUData &imu) {
  hs.word(imu.header.timestamp);
  hs.word(imu.header.sequence_number);
  for (float v : {imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz}) {
    hs.f32(v);
  }
}

template <class Field> void hashField(Hasher &hs, const Field &field) {
  hs.bytes(field.data(), static_cast<size_t>(field.size()) *
                             sizeof(typename Field::value_type));
}

void hashCloudMsg(Hasher &hs, const sensors::PointCloud3 &msg) {
  hs.word(msg.has_header());
  hs.word(msg.header().timestamp());
  hs.word(msg.header().sequence_number());
  hashField(hs, msg.x());
  hashField(hs, msg.y());
  hashField(hs, msg.z());
  hashField(hs, msg.intensity());
  hs.word(static_cast<uint64_t>(msg.r_size()) << 40 |
          static_cast<uint64_t>(msg.g_size()) << 20 |
          static_cast<uint64_t>(msg.b_size()));
}

void hashImuMsg(Hasher &hs, const sensors::IMUData &msg) {
  hs.word(msg.has_header());
  hs.word(msg.header().timestamp());
  hs.word(msg.header().sequence_number());
  for (float v : {msg.ax(), msg.ay(), msg.az(), msg.gx(), msg.gy(), msg.gz()}) {
    hs.f32(v);
  }
}

uint64_t hashFile(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  std::vector<char> buffer(1 << 20);
  Hasher hs;
  while (in) {
    in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto n = static_cast<size_t>(in.gcount());
    if (n > 0) {
      hs.bytes(buffer.data(), n);
    }
  }
  return hs.h;
}

bool filesEqual(const std::filesystem::path &a,
                const std::filesystem::path &b) {
  if (std::filesystem::file_size(a) != std::filesystem::file_size(b)) {
    return false;
  }
  std::ifstream fa(a, std::ios::binary);
  std::ifstream fb(b, std::ios::binary);
  std::vector<char> ba(1 << 20);
  std::vector<char> bb(1 << 20);
  while (fa && fb) {
    fa.read(ba.data(), static_cast<std::streamsize>(ba.size()));
    fb.read(bb.data(), static_cast<std::streamsize>(bb.size()));
    if (fa.gcount() != fb.gcount() ||
        std::memcmp(ba.data(), bb.data(), static_cast<size_t>(fa.gcount())) !=
            0) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Test data.

struct Recording {
  std::string name;
  std::filesystem::path path;
  std::vector<sensors::PointCloud3> scan_msgs;
  std::vector<msensor::Scan3DI> scans;
  std::vector<sensors::IMUData> imu_msgs;
  std::vector<msensor::IMUData> imus;
  /// File order: true = scan, false = IMU.
  std::vector<bool> order;
  size_t points = 0;
};

Recording loadRecording(const std::filesystem::path &path) {
  Recording rec;
  rec.path = path;
  rec.name = path.stem().string();
  msensor::ScanPlayer player(path);
  while (player.next()) {
    const auto &entry = player.getLastEntry();
    if (entry.entry_case() == sensors::RecordingEntry::kScan) {
      rec.scan_msgs.push_back(entry.scan());
      rec.scans.push_back(fromProtobuf(entry.scan()));
      rec.points += rec.scans.back().points.size();
      rec.order.push_back(true);
    } else if (entry.entry_case() == sensors::RecordingEntry::kImu) {
      rec.imu_msgs.push_back(entry.imu());
      rec.imus.push_back(fromProtobuf(entry.imu()));
      rec.order.push_back(false);
    }
  }
  return rec;
}

/// Deterministic synthetic scan; intensities are reflectivity-like integers.
msensor::Scan3DI makeSyntheticScan(size_t points, uint32_t seed) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> coord(-30.0F, 30.0F);
  std::uniform_int_distribution<int> reflectivity(0, 255);
  msensor::Scan3DI scan;
  scan.header.timestamp = 1'000'000'000ULL + seed;
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

// ---------------------------------------------------------------------------
// Row runner.

struct PassResult {
  double seconds = 0.0;
  size_t ops = 0;
  uint64_t checksum = 0;
};

struct Options {
  int passes = 3;
  size_t grpc_scans = 200;
  size_t grpc_imus = 2000;
  std::filesystem::path tmp = std::filesystem::temp_directory_path();
  std::string only;
  std::vector<std::filesystem::path> recordings;
};

const Options *g_options = nullptr;

void runRow(const std::string &name, const std::function<PassResult()> &pass) {
  if (!g_options->only.empty() &&
      name.find(g_options->only) == std::string::npos) {
    return;
  }
  double best = std::numeric_limits<double>::infinity();
  size_t ops = 0;
  uint64_t checksum = 0;
  bool stable = true;
  for (int i = 0; i < g_options->passes; ++i) {
    const PassResult r = pass();
    if (i == 0) {
      checksum = r.checksum;
      ops = r.ops;
    } else if (r.checksum != checksum || r.ops != ops) {
      stable = false;
    }
    best = std::min(best, r.seconds);
  }
  std::printf("ROW %-40s %10.3f %10.3f %8zu %016llx%s\n", name.c_str(),
              best * 1e3, ops ? best * 1e6 / static_cast<double>(ops) : 0.0,
              ops, static_cast<unsigned long long>(checksum),
              stable ? "" : " UNSTABLE");
  std::fflush(stdout);
}

// ---------------------------------------------------------------------------
// Conversions.

PassResult scansToProto(const std::vector<msensor::Scan3DI> &scans,
                        int repeat) {
  PassResult r;
  Hasher hs;
  sensors::PointCloud3 msg;
  for (int k = 0; k < repeat; ++k) {
    for (const auto &scan : scans) {
      const auto t0 = Clock::now();
      toProtobuf(scan, msg);
      r.seconds += Seconds(Clock::now() - t0).count();
      if (k == 0) {
        hashCloudMsg(hs, msg);
      }
      ++r.ops;
    }
  }
  r.checksum = hs.h;
  return r;
}

PassResult scansFromProto(const std::vector<sensors::PointCloud3> &msgs,
                          int repeat) {
  PassResult r;
  Hasher hs;
  for (int k = 0; k < repeat; ++k) {
    for (const auto &msg : msgs) {
      const auto t0 = Clock::now();
      auto scan = fromProtobuf(msg);
      r.seconds += Seconds(Clock::now() - t0).count();
      if (k == 0) {
        hashScan(hs, scan);
      }
      ++r.ops;
    }
  }
  r.checksum = hs.h;
  return r;
}

PassResult imusToProto(const std::vector<msensor::IMUData> &imus) {
  PassResult r;
  Hasher hs;
  sensors::IMUData msg;
  const auto t0 = Clock::now();
  for (const auto &imu : imus) {
    toProtobuf(imu, msg);
    hashImuMsg(hs, msg);
  }
  r.seconds = Seconds(Clock::now() - t0).count();
  r.ops = imus.size();
  r.checksum = hs.h;
  return r;
}

PassResult imusFromProto(const std::vector<sensors::IMUData> &msgs) {
  PassResult r;
  Hasher hs;
  const auto t0 = Clock::now();
  for (const auto &msg : msgs) {
    hashImu(hs, fromProtobuf(msg));
  }
  r.seconds = Seconds(Clock::now() - t0).count();
  r.ops = msgs.size();
  r.checksum = hs.h;
  return r;
}

// ---------------------------------------------------------------------------
// Recording playback and recording.

/// A recording on disk, for the rows that read the file themselves.
struct RecordingFile {
  std::string name;
  std::filesystem::path path;
  size_t entries = 0;
};

RecordingFile countEntries(const std::filesystem::path &path) {
  RecordingFile file{path.stem().string(), path, 0};
  msensor::ScanPlayer player(path);
  while (player.next()) {
    ++file.entries;
  }
  return file;
}

PassResult playerParse(const RecordingFile &rec) {
  PassResult r;
  Hasher hs;
  msensor::ScanPlayer player(rec.path);
  while (true) {
    const auto t0 = Clock::now();
    const bool more = player.next();
    r.seconds += Seconds(Clock::now() - t0).count();
    if (!more) {
      break;
    }
    const auto &entry = player.getLastEntry();
    hs.word(static_cast<uint64_t>(entry.entry_case()));
    if (entry.entry_case() == sensors::RecordingEntry::kScan) {
      hashCloudMsg(hs, entry.scan());
    } else if (entry.entry_case() == sensors::RecordingEntry::kImu) {
      hashImuMsg(hs, entry.imu());
    }
    ++r.ops;
  }
  r.checksum = hs.h;
  return r;
}

/// Unpaced RecordingSensorDriver playback into a consumer that takes
/// ownership of every scan (as a SLAM front end queueing scans does). With
/// `full_hash` the consumer hashes every point; otherwise it records only the
/// headers and sizes so the row measures the driver itself.
PassResult playbackUnpaced(const RecordingFile &rec, bool full_hash) {
  PassResult r;
  const size_t expected = rec.entries;
  msensor::RecordingSensorDriver driver(rec.path, 0.0);

  std::mutex m;
  std::condition_variable cv;
  size_t emitted = 0; // only touched on the playback thread until done
  bool done = false;
  Clock::time_point end;
  Hasher hs;
  msensor::Scan3DI owned;

  const auto count = [&]() {
    if (++emitted == expected) {
      end = Clock::now();
      std::lock_guard lock(m);
      done = true;
      cv.notify_all();
    }
  };
  driver.setScanCallback([&](msensor::Scan3DI scan) {
    if (full_hash) {
      hashScan(hs, scan);
    } else {
      hs.word(scan.header.timestamp);
      hs.word(scan.points.size());
    }
    owned = std::move(scan);
    count();
  });
  driver.setImuCallback([&](msensor::IMUData imu) {
    hashImu(hs, imu);
    count();
  });

  const auto start = Clock::now();
  driver.startSampling();
  {
    std::unique_lock lock(m);
    if (!cv.wait_for(lock, std::chrono::minutes(5), [&] { return done; })) {
      std::fprintf(stderr, "playback of %s timed out after %zu/%zu entries\n",
                   rec.name.c_str(), emitted, expected);
      std::exit(2);
    }
  }
  driver.stopSampling();
  r.seconds = Seconds(end - start).count();
  r.ops = expected;
  r.checksum = hs.h;
  return r;
}

PassResult recordAll(const Recording &rec, bool *identical) {
  PassResult r;
  const auto out = g_options->tmp / ("bench_" + rec.name + ".pbscan");
  {
    auto file = std::make_shared<msensor::File>();
    msensor::ScanRecorder recorder(file);
    recorder.start(out.string());
    size_t s = 0;
    size_t i = 0;
    const auto t0 = Clock::now();
    for (const bool is_scan : rec.order) {
      if (is_scan) {
        recorder.record(rec.scans[s++]);
      } else {
        recorder.record(rec.imus[i++]);
      }
    }
    recorder.stop();
    r.seconds = Seconds(Clock::now() - t0).count();
    r.ops = rec.order.size();
  }
  r.checksum = hashFile(out);
  *identical = filesEqual(out, rec.path);
  std::filesystem::remove(out);
  return r;
}

// ---------------------------------------------------------------------------
// gRPC: a driver stand-in that emits on demand, served by SensorsServer and
// consumed through SensorsRemoteClient. Each sample is sent only after the
// previous one has reached the client callback (ping-pong), so no sample is
// skipped by the server's latest-wins policy and every row is deterministic.

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
  void push(msensor::Scan3DI scan) { scans_.emit(std::move(scan)); }
  void push(msensor::IMUData imu) { imus_.emit(imu); }

private:
  msensor::CallbackSlot<msensor::Scan3DI> scans_;
  msensor::CallbackSlot<msensor::IMUData> imus_;
};

template <class T> struct Mailbox {
  std::mutex m;
  std::condition_variable cv;
  bool full = false;
  T value{};

  void put(T v) {
    std::lock_guard lock(m);
    value = std::move(v);
    full = true;
    cv.notify_all();
  }

  /// Wait for a value; false on timeout.
  bool take(T &out, std::chrono::milliseconds timeout) {
    std::unique_lock lock(m);
    if (!cv.wait_for(lock, timeout, [&] { return full; })) {
      return false;
    }
    out = std::move(value);
    full = false;
    return true;
  }
};

struct GrpcRig {
  std::shared_ptr<FeedSensor> feed = std::make_shared<FeedSensor>();
  std::unique_ptr<SensorsServer> server;
  std::unique_ptr<SensorsRemoteClient> client;
  Mailbox<msensor::Scan3DI> scans;
  Mailbox<msensor::IMUData> imus;

  explicit GrpcRig(const std::string &address) {
    server =
        std::make_unique<SensorsServer>(nullptr, nullptr, feed, feed, address);
    server->start();
    client = std::make_unique<SensorsRemoteClient>(address);
    client->setScanCallback(
        [this](msensor::Scan3DI scan) { scans.put(std::move(scan)); });
    client->setImuCallback([this](msensor::IMUData imu) { imus.put(imu); });
    client->start();
  }

  ~GrpcRig() {
    client->stop();
    server->stop();
  }

  /// Push warm-up samples until both streams deliver.
  void connect(const msensor::Scan3DI &scan, const msensor::IMUData &imu) {
    for (int attempt = 0; attempt < 500; ++attempt) {
      feed->push(scan);
      msensor::Scan3DI got;
      if (scans.take(got, std::chrono::milliseconds(20))) {
        break;
      }
    }
    for (int attempt = 0; attempt < 500; ++attempt) {
      feed->push(imu);
      msensor::IMUData got;
      if (imus.take(got, std::chrono::milliseconds(20))) {
        return;
      }
    }
    std::fprintf(stderr, "gRPC rig failed to connect\n");
    std::exit(2);
  }
};

template <class T> bool sameHeader(const T &a, const T &b) {
  return a.header.timestamp == b.header.timestamp &&
         a.header.sequence_number == b.header.sequence_number;
}

/// Wait for `expected` to arrive, discarding stale warm-up deliveries.
template <class T>
T receive(Mailbox<T> &mailbox, const T &expected, const char *what) {
  T received;
  do {
    if (!mailbox.take(received, std::chrono::seconds(10))) {
      std::fprintf(stderr, "gRPC %s ping-pong timed out\n", what);
      std::exit(2);
    }
  } while (!sameHeader(received, expected));
  return received;
}

PassResult grpcScanPingPong(GrpcRig &rig,
                            const std::vector<msensor::Scan3DI> &scans) {
  PassResult r;
  Hasher hs;
  for (const auto &original : scans) {
    msensor::Scan3DI to_send = original; // copy outside the timed region
    const auto t0 = Clock::now();
    rig.feed->push(std::move(to_send));
    const auto received = receive(rig.scans, original, "scan");
    r.seconds += Seconds(Clock::now() - t0).count();
    hashScan(hs, received);
    ++r.ops;
  }
  r.checksum = hs.h;
  return r;
}

PassResult grpcImuPingPong(GrpcRig &rig,
                           const std::vector<msensor::IMUData> &imus) {
  PassResult r;
  Hasher hs;
  const auto t0 = Clock::now();
  for (const auto &imu : imus) {
    rig.feed->push(imu);
    hashImu(hs, receive(rig.imus, imu, "IMU"));
    ++r.ops;
  }
  r.seconds = Seconds(Clock::now() - t0).count();
  r.checksum = hs.h;
  return r;
}

// ---------------------------------------------------------------------------
// Camera conversion: raw frame (JPEG-encoded on conversion) vs. a frame that
// is already JPEG (passed through).

/// Deterministic 1536x864 BGR image: gradient plus fixed-seed noise.
cv::Mat makeSyntheticImage() {
  constexpr int kWidth = 1536;
  constexpr int kHeight = 864;
  cv::Mat image(kHeight, kWidth, CV_8UC3);
  std::mt19937 gen(1234);
  std::uniform_int_distribution<int> noise(-12, 12);
  const auto clamp8 = [](int v) {
    return static_cast<uchar>(std::clamp(v, 0, 255));
  };
  for (int y = 0; y < kHeight; ++y) {
    auto *row = image.ptr<cv::Vec3b>(y);
    for (int x = 0; x < kWidth; ++x) {
      row[x] = cv::Vec3b(clamp8(x * 255 / kWidth + noise(gen)),
                         clamp8(y * 255 / kHeight + noise(gen)),
                         clamp8((x + y) * 255 / (kWidth + kHeight) + noise(gen)));
    }
  }
  return image;
}

PassResult cameraToProto(const msensor::CameraFrame &frame, int repeat) {
  PassResult r;
  Hasher hs;
  sensors::CameraStreamReply msg;
  const auto t0 = Clock::now();
  for (int i = 0; i < repeat; ++i) {
    toProtobuf(frame, msg);
    hs.bytes(msg.image_data().data(), msg.image_data().size());
    hs.word(msg.width());
    hs.word(msg.height());
    hs.word(static_cast<uint64_t>(msg.encoding()));
    ++r.ops;
  }
  r.seconds = Seconds(Clock::now() - t0).count();
  r.checksum = hs.h;
  return r;
}

void usage(const char *program) {
  std::fprintf(stderr,
               "Usage: %s [--passes N] [--grpc-scans N] [--grpc-imus N] "
               "[--tmp DIR] [--only SUBSTR] [recording.pbscan ...]\n",
               program);
}

} // namespace

int main(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    const auto value = [&]() -> const char * {
      if (i + 1 >= argc) {
        usage(argv[0]);
        std::exit(1);
      }
      return argv[++i];
    };
    if (arg == "--passes") {
      options.passes = std::max(1, std::atoi(value()));
    } else if (arg == "--grpc-scans") {
      options.grpc_scans = std::strtoul(value(), nullptr, 10);
    } else if (arg == "--grpc-imus") {
      options.grpc_imus = std::strtoul(value(), nullptr, 10);
    } else if (arg == "--tmp") {
      options.tmp = value();
    } else if (arg == "--only") {
      options.only = value();
    } else if (arg == "-h" || arg == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      options.recordings.emplace_back(arg);
    }
  }
  g_options = &options;

  // Rows that read the recording files run first, while the heap is small:
  // with the in-memory copies of the recordings loaded, the allocations in
  // the parser take about twice as long, which would distort these rows.
  for (const auto &path : options.recordings) {
    const RecordingFile file = countEntries(path);
    const std::string suffix = "[" + file.name + "]";
    runRow("player_next" + suffix, [&] { return playerParse(file); });
    runRow("playback_unpaced" + suffix,
           [&] { return playbackUnpaced(file, false); });
    runRow("playback_content" + suffix,
           [&] { return playbackUnpaced(file, true); });
  }

  // Synthetic scans: SimLidar size, one Mid360 scan at the default 100
  // packets x 96 points, and a dense cloud.
  for (const size_t points : {size_t{2000}, size_t{9600}, size_t{100000}}) {
    std::vector<msensor::Scan3DI> scans;
    std::vector<sensors::PointCloud3> msgs;
    for (uint32_t seed = 1; seed <= 8; ++seed) {
      scans.push_back(makeSyntheticScan(points, seed));
      msgs.emplace_back();
      toProtobuf(scans.back(), msgs.back());
    }
    const int repeat = static_cast<int>(std::max<size_t>(1, 400000 / points));
    const std::string suffix = "[synthetic " + std::to_string(points) + "]";
    runRow("scan_to_proto" + suffix,
           [&] { return scansToProto(scans, repeat); });
    runRow("scan_from_proto" + suffix,
           [&] { return scansFromProto(msgs, repeat); });
  }

  {
    msensor::CameraFrame raw;
    raw.header = {1, 1};
    raw.mat = makeSyntheticImage();
    msensor::CameraFrame jpeg;
    jpeg.header = {1, 1};
    jpeg.encoding = msensor::CameraFrameEncoding::Jpeg;
    std::vector<uchar> bytes;
    cv::imencode(".jpg", raw.mat, bytes, {cv::IMWRITE_JPEG_QUALITY, 85});
    jpeg.mat = cv::Mat(1, static_cast<int>(bytes.size()), CV_8UC1,
                       bytes.data())
                   .clone();
    runRow("camera_to_proto_raw[synthetic 1536x864]",
           [&] { return cameraToProto(raw, 20); });
    runRow("camera_to_proto_jpeg[synthetic 1536x864]",
           [&] { return cameraToProto(jpeg, 200); });
  }

  int port = 50061;
  for (const auto &path : options.recordings) {
    const auto load_start = Clock::now();
    const Recording rec = loadRecording(path);
    std::printf("# %s: %zu entries, %zu scans, %zu IMU samples, %zu points "
                "(%.1f per scan), loaded in %.2f s\n",
                rec.name.c_str(), rec.order.size(), rec.scans.size(),
                rec.imus.size(), rec.points,
                rec.scans.empty() ? 0.0
                                  : static_cast<double>(rec.points) /
                                        static_cast<double>(rec.scans.size()),
                Seconds(Clock::now() - load_start).count());
    const std::string suffix = "[" + rec.name + "]";

    runRow("scan_to_proto" + suffix,
           [&] { return scansToProto(rec.scans, 1); });
    runRow("scan_from_proto" + suffix,
           [&] { return scansFromProto(rec.scan_msgs, 1); });
    runRow("imu_to_proto" + suffix, [&] { return imusToProto(rec.imus); });
    runRow("imu_from_proto" + suffix,
           [&] { return imusFromProto(rec.imu_msgs); });

    bool identical = false;
    runRow("record" + suffix, [&] { return recordAll(rec, &identical); });
    if (options.only.empty() ||
        std::string("record").find(options.only) != std::string::npos) {
      std::printf("# re-recorded %s is byte-identical to the source: %s\n",
                  rec.name.c_str(), identical ? "yes" : "NO");
    }

    const bool want_grpc = options.only.empty() ||
                           std::string("grpc_scan_pingpong grpc_imu_pingpong")
                                   .find(options.only) != std::string::npos;
    if (want_grpc && !rec.scans.empty() && !rec.imus.empty()) {
      GrpcRig rig("127.0.0.1:" + std::to_string(port++));
      rig.connect(rec.scans.front(), rec.imus.front());
      const std::vector<msensor::Scan3DI> scans(
          rec.scans.begin(),
          rec.scans.begin() + static_cast<std::ptrdiff_t>(std::min(
                                  rec.scans.size(), options.grpc_scans)));
      const std::vector<msensor::IMUData> imus(
          rec.imus.begin(),
          rec.imus.begin() + static_cast<std::ptrdiff_t>(
                                 std::min(rec.imus.size(), options.grpc_imus)));
      runRow("grpc_scan_pingpong" + suffix,
             [&] { return grpcScanPingPong(rig, scans); });
      runRow("grpc_imu_pingpong" + suffix,
             [&] { return grpcImuPingPong(rig, imus); });
    }
  }
  return 0;
}
