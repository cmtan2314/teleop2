#include "quest_vr/logging.hpp"

#include <atomic>
#include <iostream>
#include <mutex>

namespace quest_vr {
namespace {

std::mutex g_sink_mutex;
LogSink g_sink;  // empty → default stderr sink
std::atomic<LogLevel> g_min_level{LogLevel::kInfo};

const char* levelName(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug:
      return "DEBUG";
    case LogLevel::kInfo:
      return "INFO";
    case LogLevel::kWarn:
      return "WARN";
    case LogLevel::kError:
      return "ERROR";
  }
  return "?";
}

}  // namespace

void setLogSink(LogSink sink) {
  const std::lock_guard<std::mutex> lock(g_sink_mutex);
  g_sink = std::move(sink);
}

void setMinLogLevel(LogLevel level) { g_min_level.store(level); }

void logMessage(LogLevel level, const std::string& message) {
  if (level < g_min_level.load()) {
    return;
  }
  const std::lock_guard<std::mutex> lock(g_sink_mutex);
  if (g_sink) {
    g_sink(level, message);
    return;
  }
  std::cerr << "[receiver][" << levelName(level) << "] " << message << '\n';
}

}  // namespace quest_vr
