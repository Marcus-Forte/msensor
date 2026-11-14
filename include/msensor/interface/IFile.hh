
#pragma once

#include <cstddef>
#include <string>

namespace msensor {

/**
 * @brief Adapter to a file interface.
 *
 */
class IFile {
public:
  virtual ~IFile() = default;

  /// Open the underlying file for writing.
  virtual void open(const std::string &filename) = 0;
  /// Write a raw buffer to the file. Throws std::runtime_error if the
  /// underlying file is (or becomes) failed.
  virtual void write(const char *data, size_t size) = 0;
  /// Push buffered data to the OS. Throws std::runtime_error if the underlying
  /// file is (or becomes) failed.
  virtual void flush() = 0;
  /// Flush and close the file.
  virtual void close() = 0;
};

} // namespace msensor