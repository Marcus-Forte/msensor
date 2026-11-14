#pragma once

// Minimal logging facade for msensor library code (header-only; the state is
// held in inline functions, so every library shares one instance per program).
//
//   msensor::log::info("Listening on {}...", address);
//
// The threshold defaults to info and can be preset with the environment
// variable MSENSOR_LOG_LEVEL (debug, info, warn, error, off; case-insensitive),
// which is read once on first use.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace msensor::log {

/// Severity of a message. `off` is only meaningful as a threshold.
enum class Level : int { debug = 0, info, warn, error, off };

/// Receives every message at or above the threshold, already formatted and
/// without a trailing newline.
using Sink = std::function<void(Level, std::string_view message)>;

/// Parses "debug", "info", "warn", "error" or "off" (case-insensitive).
inline std::optional<Level> parseLevel(std::string_view text) {
  std::string lower(text);
  std::ranges::transform(lower, lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (lower == "debug") return Level::debug;
  if (lower == "info") return Level::info;
  if (lower == "warn") return Level::warn;
  if (lower == "error") return Level::error;
  if (lower == "off") return Level::off;
  return std::nullopt;
}

/// Name used as the prefix of the default sink.
inline std::string_view levelName(Level level) {
  switch (level) {
  case Level::debug: return "debug";
  case Level::info: return "info";
  case Level::warn: return "warn";
  case Level::error: return "error";
  case Level::off: return "off";
  }
  return "?";
}

namespace detail {

struct State {
  std::atomic<int> level{static_cast<int>(Level::info)};
  std::mutex mutex; // serialises sink invocations and sink replacement
  Sink sink;        // empty: default sink
  std::string rejected_env; // unrecognised MSENSOR_LOG_LEVEL, reported once

  State() {
    if (const char *env = std::getenv("MSENSOR_LOG_LEVEL")) {
      if (const auto parsed = parseLevel(env)) {
        level.store(static_cast<int>(*parsed));
      } else {
        rejected_env = env;
      }
    }
  }
};

inline void defaultSink(Level level, std::string_view message) {
  std::FILE *out = level >= Level::warn ? stderr : stdout;
  const auto name = levelName(level);
  const std::string line = std::format("[{}] {}\n", name, message);
  std::fwrite(line.data(), 1, line.size(), out);
  std::fflush(out);
}

/// Delivers a message without consulting the threshold.
inline void write(State &state, Level level, std::string_view message) {
  std::lock_guard lock(state.mutex);
  if (state.sink) {
    state.sink(level, message);
  } else {
    defaultSink(level, message);
  }
}

inline State &state() {
  static State instance;
  // Runs once, after the state is complete; write() does not re-enter state().
  static const bool reported = [] {
    if (!instance.rejected_env.empty() &&
        instance.level.load() <= static_cast<int>(Level::warn)) {
      write(instance, Level::warn,
            std::format("Ignoring unrecognised MSENSOR_LOG_LEVEL '{}'",
                        instance.rejected_env));
    }
    return true;
  }();
  (void)reported;
  return instance;
}

template <typename... Args>
void emit(Level level, std::format_string<Args...> fmt, Args &&...args) {
  State &s = state();
  if (static_cast<int>(level) < s.level.load(std::memory_order_relaxed)) {
    return; // below the threshold: nothing is formatted
  }
  write(s, level, std::format(fmt, std::forward<Args>(args)...));
}

} // namespace detail

/// Sets the threshold; thread-safe.
inline void setLevel(Level level) {
  detail::state().level.store(static_cast<int>(level));
}

/// Current threshold; thread-safe.
inline Level level() {
  return static_cast<Level>(detail::state().level.load());
}

/// Replaces the sink; an empty function restores the default sink (one
/// "[level] message" line, debug/info on stdout, warn/error on stderr, each
/// flushed). Sink invocations are serialised, so a sink need not be
/// thread-safe, and it MUST NOT log (that would deadlock) or call setSink().
inline void setSink(Sink sink) {
  auto &s = detail::state();
  std::lock_guard lock(s.mutex);
  s.sink = std::move(sink);
}

template <typename... Args>
void debug(std::format_string<Args...> fmt, Args &&...args) {
  detail::emit(Level::debug, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
void info(std::format_string<Args...> fmt, Args &&...args) {
  detail::emit(Level::info, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
void warn(std::format_string<Args...> fmt, Args &&...args) {
  detail::emit(Level::warn, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
void error(std::format_string<Args...> fmt, Args &&...args) {
  detail::emit(Level::error, fmt, std::forward<Args>(args)...);
}

} // namespace msensor::log
