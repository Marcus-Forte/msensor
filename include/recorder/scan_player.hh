#pragma once

#include <filesystem>

#include <sensors.pb.h>

namespace msensor {

/**
 * @brief Sequentially replays recorded scans from disk.
 */
class ScanPlayer {
public:
  /// Open a recording file for playback.
  ScanPlayer(const std::filesystem::path &file);

  /// Advance to the next entry; returns false on end-of-file.
  bool next();
  /// Retrieve the last decoded entry.
  const sensors::RecordingEntry &getLastEntry();

private:
  char *memory_map_;
  size_t offset_;
  size_t num_bytes_;
  sensors::RecordingEntry entry_;
};
} // namespace msensor