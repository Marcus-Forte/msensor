#include "msensor/recorder/scan_player.hh"
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace msensor {

ScanPlayer::ScanPlayer(const std::filesystem::path &file) {
  const int fd = open(file.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd == -1) {
    if (errno == ENOENT) {
      throw std::runtime_error("File does not exist: " + file.string());
    }
    throw std::runtime_error("Failed to open recording " + file.string() +
                             ": " + std::strerror(errno));
  }

  struct stat file_stat{};
  if (fstat(fd, &file_stat) != 0) {
    const int error = errno;
    close(fd);
    throw std::runtime_error("Failed to stat recording " + file.string() +
                             ": " + std::strerror(error));
  }
  num_bytes_ = static_cast<std::size_t>(file_stat.st_size);

  // mmap() rejects empty mappings; an empty file simply has no entries.
  if (num_bytes_ > 0) {
    void *memmap = mmap(nullptr, num_bytes_, PROT_READ, MAP_PRIVATE, fd, 0);
    if (memmap == MAP_FAILED) {
      close(fd);
      throw std::runtime_error("Failed to map file into memory.");
    }
    memory_map_ = static_cast<const char *>(memmap);
  }
  // The mapping stays valid after the descriptor is closed.
  close(fd);
}

ScanPlayer::~ScanPlayer() {
  if (memory_map_ != nullptr) {
    munmap(const_cast<char *>(memory_map_), num_bytes_);
  }
}

bool ScanPlayer::next() {
  if (offset_ >= num_bytes_) {
    return false;
  }
  std::uint64_t msg_size;
  if (num_bytes_ - offset_ < sizeof(msg_size)) {
    throw std::runtime_error("Truncated recording entry size.");
  }
  std::memcpy(&msg_size, memory_map_ + offset_, sizeof(msg_size));
  offset_ += sizeof(msg_size);
  if (msg_size > num_bytes_ - offset_ ||
      msg_size > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
    throw std::runtime_error("Invalid recording entry size.");
  }
  if (!entry_.ParseFromArray(memory_map_ + offset_,
                             static_cast<int>(msg_size))) {
    throw std::runtime_error("Failed to parse recording entry.");
  }
  offset_ += msg_size;
  return true;
}

void ScanPlayer::reset() { offset_ = 0; }

const sensors::RecordingEntry &ScanPlayer::getLastEntry() const {
  return entry_;
}

} // namespace msensor
