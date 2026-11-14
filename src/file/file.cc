#include "msensor/file/file.hh"

#include <stdexcept>

namespace msensor {
File::File() = default;

File::~File() = default;

void File::open(const std::string &filename) {
  file_.open(filename, std::ios::binary | std::ios::trunc);
  if (!file_) {
    throw std::runtime_error("Unable to open file for writing: " + filename);
  }
}

void File::close() { file_.close(); }

void File::write(const char *data, size_t size) {
  file_.write(data, static_cast<std::streamsize>(size));
  if (!file_) {
    throw std::runtime_error("Failed to write to file.");
  }
}

void File::flush() {
  file_.flush();
  if (!file_) {
    throw std::runtime_error("Failed to flush file.");
  }
}

} // namespace msensor