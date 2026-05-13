#include "msensor/recorder/scan_recorder.hh"
#include "msensor/conversions/conversions.hh"
#include "msensor/timing/timing.hh"
#include "recording.pb.h"
#include <mutex>

std::mutex g_mutex;

namespace msensor {

ScanRecorder::ScanRecorder(const std::shared_ptr<IFile> &file)
    : record_file_{file}, has_started_{false} {}

ScanRecorder::~ScanRecorder() { record_file_->close(); }

void ScanRecorder::start() {
  const auto cur_time = timing::getNowUs();
  filename_ = "scan_" + std::to_string(cur_time) + ".pbscan";
  record_file_->open(filename_);
  has_started_ = true;
}

void ScanRecorder::start(const std::string &filename) {
  filename_ = filename;
  record_file_->open(filename_);
  has_started_ = true;
}

void ScanRecorder::record(const std::shared_ptr<Scan3DI> &scan) {
  if (!has_started_)
    return;

  sensors::RecordingEntry entry;
  *entry.mutable_scan() = toProtobuf(scan);

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
  proto_msg->mutable_header()->set_timestamp(imu.header.timestamp);
  proto_msg->mutable_header()->set_sequence_number(imu.header.sequence_number);
  proto_msg->set_ax(imu.ax);
  proto_msg->set_ay(imu.ay);
  proto_msg->set_az(imu.az);
  proto_msg->set_gx(imu.gx);
  proto_msg->set_gy(imu.gy);
  proto_msg->set_gz(imu.gz);

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

const std::string &ScanRecorder::getFilename() const { return filename_; }
} // namespace msensor