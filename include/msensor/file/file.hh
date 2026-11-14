#pragma once

#include "msensor/interface/IFile.hh"
#include <fstream>

namespace msensor {
/**
 * @brief Filesystem-backed implementation of `IFile`.
 */
class File : public IFile {
public:
  File();
  ~File() override;
  /// Open (truncate) a file for binary output. Throws std::runtime_error if
  /// the file cannot be opened.
  void open(const std::string &filename) override;
  /// Write a buffer to the open file. Throws std::runtime_error if the file
  /// is in a failed state.
  void write(const char *data, size_t size) override;
  /// Flush to the OS. Throws std::runtime_error if the file is in a failed
  /// state.
  void flush() override;
  /// Close the current file handle.
  void close() override;

private:
  std::ofstream file_;
};
} // namespace msensor