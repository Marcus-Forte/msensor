#include "recorder/scan_recorder.hh"
#include "sensors.pb.h"
#include "timing/timing.hh"
#include <mutex>

std::mutex g_mutex;

namespace {
void toProtobuf(const std::shared_ptr<msensor::Scan3DI> &scan,
                sensors::PointCloud3 *proto_msg) {
  for (const auto &pt : *scan->points) {
    auto *proto_pt = proto_msg->add_points();
    proto_pt->set_x(pt.x);
    proto_pt->set_y(pt.y);
    proto_pt->set_z(pt.z);
  }
  proto_msg->set_timestamp(scan->timestamp);
}
} // namespace

namespace msensor {

ScanRecorder::ScanRecorder(const std::shared_ptr<IFile> &file)
    : record_file_{file}, has_started_{false} {}

ScanRecorder::~ScanRecorder() { record_file_->close(); }

void ScanRecorder::start() {
  const auto cur_time = timing::getNowUs();
  record_file_->open("scan_" + std::to_string(cur_time) + ".pbscan");
  has_started_ = true;
}

void ScanRecorder::start(const std::string &filename) {
  record_file_->open(filename);
  has_started_ = true;
}

void ScanRecorder::record(const std::shared_ptr<Scan3DI> &scan) {
  if (!has_started_)
    return;

  sensors::RecordingEntry entry;
  auto *proto_msg = entry.mutable_scan();
  toProtobuf(scan, proto_msg);

  auto bytes = entry.ByteSizeLong();
  {
    std::scoped_lock<std::mutex> lock(g_mutex);
    // Write size of data
    record_file_->write(reinterpret_cast<char *>(&bytes), sizeof(size_t));
    // Write the sensor data
    entry.SerializeToOstream(record_file_->ostream());
    *record_file_->ostream() << std::flush;
  }
}

void ScanRecorder::record(msensor::IMUData imu) {
  if (!has_started_)
    return;

  sensors::RecordingEntry entry;
  auto *proto_msg = entry.mutable_imu();
  proto_msg->set_ax(imu.ax);
  proto_msg->set_ay(imu.ay);
  proto_msg->set_az(imu.az);
  proto_msg->set_gx(imu.gx);
  proto_msg->set_gy(imu.gy);
  proto_msg->set_gz(imu.gz);
  proto_msg->set_timestamp(imu.timestamp);

  auto bytes = entry.ByteSizeLong();

  {
    std::scoped_lock<std::mutex> lock(g_mutex);
    // Write size of data
    record_file_->write(reinterpret_cast<char *>(&bytes), sizeof(size_t));
    // Write the sensor data
    entry.SerializeToOstream(record_file_->ostream());
    *record_file_->ostream() << std::flush;
  }
}

void ScanRecorder::stop() {
  record_file_->close();
  has_started_ = false;
}
} // namespace msensor