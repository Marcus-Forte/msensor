#include "msensor/recorder/scan_recorder.hh"
#include "msensor/conversions/conversions.hh"
#include "msensor/timing/timing.hh"
#include "recording.pb.h"
#include <stdexcept>

namespace msensor {

ScanRecorder::ScanRecorder(const std::shared_ptr<IFile> &file)
    : record_file_{file},
      scan_entry_{std::make_unique<sensors::RecordingEntry>()},
      imu_entry_{std::make_unique<sensors::RecordingEntry>()} {}

ScanRecorder::~ScanRecorder() { record_file_->close(); }

void ScanRecorder::start() {
  start("scan_" + std::to_string(timing::getNowUs()) + ".pbscan");
}

void ScanRecorder::start(const std::string &filename) {
  std::lock_guard lock(mutex_);
  filename_ = filename;
  record_file_->open(filename_);
  has_started_ = true;
}

void ScanRecorder::record(const Scan3DI &scan) {
  std::lock_guard lock(mutex_);
  if (!has_started_) {
    return;
  }
  toProtobuf(scan, *scan_entry_->mutable_scan());
  append(*scan_entry_);
}

void ScanRecorder::record(const IMUData &imu) {
  std::lock_guard lock(mutex_);
  if (!has_started_) {
    return;
  }
  toProtobuf(imu, *imu_entry_->mutable_imu());
  append(*imu_entry_);
}

void ScanRecorder::append(const sensors::RecordingEntry &entry) {
  // ByteSizeLong() caches the sub-message sizes, so serializing with the
  // cached sizes walks the (O(n) for packed varints) fields only once.
  const std::uint64_t bytes = entry.ByteSizeLong();
  if (buffer_.size() < bytes) {
    buffer_.resize(bytes);
  }
  const std::uint8_t *end =
      entry.SerializeWithCachedSizesToArray(buffer_.data());
  if (static_cast<std::uint64_t>(end - buffer_.data()) != bytes) {
    throw std::runtime_error("Recording entry size changed while serializing.");
  }

  // Write size of data
  record_file_->write(reinterpret_cast<const char *>(&bytes), sizeof(bytes));
  // Write the sensor data
  std::ostream *out = record_file_->ostream();
  out->write(reinterpret_cast<const char *>(buffer_.data()),
             static_cast<std::streamsize>(bytes));
  out->flush();
  if (!*out) {
    throw std::runtime_error("Failed to write recording entry.");
  }
}

void ScanRecorder::stop() {
  std::lock_guard lock(mutex_);
  record_file_->close();
  has_started_ = false;
}

const std::string &ScanRecorder::getFilename() const { return filename_; }
} // namespace msensor
