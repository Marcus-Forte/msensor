#include "msensor/recorder/scan_player.hh"
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace msensor {

ScanPlayer::ScanPlayer(const std::filesystem::path &file) {

  if (!std::filesystem::exists(file)) {
    throw std::runtime_error("File does not exist: " + file.string());
  }

  num_bytes_ = std::filesystem::file_size(file);

  int fd = open(file.c_str(), O_RDONLY);
  if (fd == -1) {
    throw std::runtime_error("Failed to open file descriptor for mmap.\n");
  }

  void *memmap = mmap(nullptr, num_bytes_, PROT_READ, MAP_PRIVATE, fd, 0);
  if (memmap == MAP_FAILED) {
    close(fd);
    throw std::runtime_error("Failed to map file into memory.");
  }
  memory_map_ = reinterpret_cast<char *>(memmap);
  offset_ = 0;
}

bool ScanPlayer::next() {
  if (offset_ < num_bytes_) {
    size_t msg_size;
    if (num_bytes_ - offset_ < sizeof(msg_size)) {
      throw std::runtime_error("Truncated recording entry size.");
    }
    std::memcpy(&msg_size, memory_map_ + offset_, sizeof(msg_size));
    offset_ += sizeof(msg_size);
    if (msg_size > num_bytes_ - offset_ ||
        msg_size > static_cast<size_t>(std::numeric_limits<int>::max())) {
      throw std::runtime_error("Invalid recording entry size.");
    }
    if (!entry_.ParseFromArray(memory_map_ + offset_,
                               static_cast<int>(msg_size))) {
      throw std::runtime_error("Failed to parse recording entry.");
    }
    offset_ += msg_size;
    return true;
  }
  return false;
}

void ScanPlayer::reset() { offset_ = 0; }

const sensors::RecordingEntry &ScanPlayer::getLastEntry() { return entry_; }

} // namespace msensor
