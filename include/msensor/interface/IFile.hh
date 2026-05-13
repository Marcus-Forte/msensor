
#pragma once

#include <cstddef>
#include <iostream>
#include <string>

namespace msensor {

/**
 * @brief Adapter to a file interface.
 *
 */
class IFile {
public:
  /// Open the underlying file for writing.
  virtual void open(const std::string &filename) = 0;
  /// Write a raw buffer to the file.
  virtual void write(const char *data, size_t size) = 0;
  /// Flush and close the file.
  virtual void close() = 0;
  /// Access the output stream backing this file.
  virtual std::ostream *ostream() = 0;
};

} // namespace msensor