#include "recorder/scan_player.hh"
#include <fcntl.h>
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
  if (memory_map_ == MAP_FAILED) {
    close(fd);
    throw std::runtime_error("Failed to map file into memory.");
  }
  memory_map_ = reinterpret_cast<char *>(memmap);
  offset_ = 0;
}

bool ScanPlayer::next() {
  size_t msg_size;

  if (offset_ < num_bytes_) {
    msg_size = *reinterpret_cast<size_t *>(memory_map_ + offset_);
    offset_ += sizeof(msg_size);
    entry_.ParseFromArray(memory_map_ + offset_, msg_size);
    offset_ += msg_size;
    return true;
  }
  return false;
}

const sensors::RecordingEntry &ScanPlayer::getLastEntry() { return entry_; }

} // namespace msensor