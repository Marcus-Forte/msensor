#pragma once

#include "interface/IFile.hh"
#include <fstream>

namespace msensor {
/**
 * @brief Filesystem-backed implementation of `IFile`.
 */
class File : public IFile {
public:
  File();
  virtual ~File();
  /// Open a file for binary output.
  void open(const std::string &filename) override;
  /// Write a buffer to the open file.
  void write(const char *data, size_t size) override;
  /// Close the current file handle.
  void close() override;
  /// Get the underlying output stream.
  std::ostream *ostream() override;

private:
  std::ofstream file_;
};
} // namespace msensor