#include "file/File.hh"

namespace msensor {
File::File() = default;

File::~File() = default;

void File::open(const std::string &filename) {
  file_.open(filename, std::ios::binary | std::ios::trunc);
}

void File::close() { file_.close(); }

void File::write(const char *data, size_t size) {
  file_.write(data, sizeof(size_t));
}

std::ostream *File::ostream() { return &file_; }

} // namespace msensor