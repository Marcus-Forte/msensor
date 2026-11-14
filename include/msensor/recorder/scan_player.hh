#pragma once

#include <cstddef>
#include <filesystem>

#include <recording.pb.h>

namespace msensor {

/**
 * @brief Sequentially replays recorded scans from disk.
 *
 * The recording is memory-mapped read-only for the lifetime of the player.
 * Each entry on disk is a native-endian `uint64_t` byte count followed by a
 * serialized `sensors::RecordingEntry` (the format ScanRecorder writes).
 * Not thread-safe; the player is not copyable.
 */
class ScanPlayer {
public:
  /// Open a recording file for playback. Throws std::runtime_error if the file
  /// cannot be opened or mapped. An empty file is a recording with no entries.
  explicit ScanPlayer(const std::filesystem::path &file);
  ~ScanPlayer();

  ScanPlayer(const ScanPlayer &) = delete;
  ScanPlayer &operator=(const ScanPlayer &) = delete;

  /// Advance to the next entry; returns false on end-of-file. Throws
  /// std::runtime_error on a truncated or corrupt entry.
  bool next();
  /// Rewind to the first entry.
  void reset();
  /// Retrieve the last decoded entry.
  const sensors::RecordingEntry &getLastEntry() const;

private:
  const char *memory_map_ = nullptr;
  std::size_t offset_ = 0;
  std::size_t num_bytes_ = 0;
  sensors::RecordingEntry entry_;
};
} // namespace msensor
