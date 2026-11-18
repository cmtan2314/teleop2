// Minimal pluggable logger. The library does not depend on ROS 2 or Dora:
// the default sink prints to stderr; an integration layer can install its own
// sink (e.g. forwarding to RCLCPP_* macros) with setLogSink().
#pragma once

#include <cstdint>
#include <format>
#include <functional>
#include <string>
#include <utility>

namespace quest_vr {

enum class LogLevel { kDebug, kInfo, kWarn, kError };

using LogSink = std::function<void(LogLevel, const std::string&)>;

// Install a sink. Passing an empty function restores the default stderr sink.
// Thread-safe: the UDP receiver thread logs concurrently with the main thread.
void setLogSink(LogSink sink);
void setMinLogLevel(LogLevel level);
void logMessage(LogLevel level, const std::string& message);

template <typename... Args>
void logf(LogLevel level, std::format_string<Args...> fmt, Args&&... args) {
  logMessage(level, std::format(fmt, std::forward<Args>(args)...));
}

// Rate limiter for a log site inside a hot loop (e.g. the IK solve at 500 Hz ×
// 10 iterations): lets the 1st occurrence and then every Nth one through, and
// counts all of them so the message can report how many were suppressed.
// Not thread-safe; give each thread its own instance.
class LogEveryN {
 public:
  explicit LogEveryN(std::uint64_t every_n) : every_n_(every_n == 0 ? 1 : every_n) {}
  // Call once per occurrence; true when this one should be logged.
  bool shouldLog() { return (count_++ % every_n_) == 0; }
  std::uint64_t count() const { return count_; }

 private:
  std::uint64_t every_n_;
  std::uint64_t count_ = 0;
};

}  // namespace quest_vr
