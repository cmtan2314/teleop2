// Standalone runner: UDP receiver thread + a fixed-rate consumer loop that
// turns the latest packet into a TeleopFrame. The consumer loop is where the
// IK solve will plug in later; for now it only prints a status line.
//
// Usage: quest_receiver [--host 0.0.0.0] [--port 5006] [--rate-hz 500] [--no-reference-cache]
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>

#include "quest_vr/constants.hpp"
#include "quest_vr/logging.hpp"
#include "quest_vr/teleop_input.hpp"
#include "quest_vr/udp_receiver.hpp"

namespace {

using quest_vr::LogLevel;
using quest_vr::logf;
using Clock = std::chrono::steady_clock;

// Consumer tick rate; matches the dataflow's `dora/timer/millis/2`.
constexpr double kDefaultRateHz = 500.0;
constexpr double kMaxRateHz = 10000.0;
constexpr int kMaxPort = 65535;
constexpr auto kStatusPeriod = std::chrono::seconds(1);

std::atomic<bool> g_stop{false};

void handleSignal(int /*signal*/) { g_stop.store(true); }

struct Options {
  std::string host{quest_vr::kDefaultHost};
  std::uint16_t port = quest_vr::kDefaultPort;
  double rate_hz = kDefaultRateHz;
  bool reference_cache = true;
};

bool parseOptions(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    const bool has_value = i + 1 < argc;
    try {
      if (arg == "--host" && has_value) {
        options.host = argv[++i];
      } else if (arg == "--port" && has_value) {
        const int port = std::stoi(argv[++i]);
        if (port <= 0 || port > kMaxPort) {
          logf(LogLevel::kError, "--port {} out of range 1..{}", port, kMaxPort);
          return false;
        }
        options.port = static_cast<std::uint16_t>(port);
      } else if (arg == "--rate-hz" && has_value) {
        options.rate_hz = std::stod(argv[++i]);
        if (!(options.rate_hz > 0.0) || options.rate_hz > kMaxRateHz) {
          logf(LogLevel::kError, "--rate-hz {} out of range (0, {}]", options.rate_hz, kMaxRateHz);
          return false;
        }
      } else if (arg == "--no-reference-cache") {
        options.reference_cache = false;
      } else {
        logf(LogLevel::kError, "unknown or incomplete argument '{}'", arg);
        return false;
      }
    } catch (const std::exception& error) {
      logf(LogLevel::kError, "bad value for '{}': {}", arg, error.what());
      return false;
    }
  }
  return true;
}

std::string describePose(const std::optional<quest_vr::Pose>& pose) {
  if (!pose.has_value()) {
    return "none";
  }
  const auto flat = quest_vr::toFlatPose(*pose);
  return std::format("[{:+.3f} {:+.3f} {:+.3f} | {:+.3f} {:+.3f} {:+.3f} {:+.3f}]", flat[0],
                     flat[1], flat[2], flat[3], flat[4], flat[5], flat[6]);
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parseOptions(argc, argv, options)) {
    std::cerr << "usage: quest_receiver [--host H] [--port P] [--rate-hz R] "
                 "[--no-reference-cache]\n";
    return EXIT_FAILURE;
  }
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  quest_vr::UdpJsonReceiver receiver(options.host, options.port);
  if (!receiver.start()) {
    return EXIT_FAILURE;
  }
  quest_vr::QuestTeleopInput teleop(quest_vr::QuestPoseProcessor(
      options.reference_cache ? quest_vr::defaultReferenceCachePath() : std::nullopt));

  const auto period = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / options.rate_hz));
  const auto t0 = Clock::now();
  auto next_tick = t0;
  auto next_status = t0 + kStatusPeriod;
  std::uint64_t ticks = 0;
  std::uint64_t last_sequence = 0;
  std::uint64_t fresh_packets = 0;
  logf(LogLevel::kInfo, "consumer loop at {:.0f} Hz", options.rate_hz);

  while (!g_stop.load()) {
    next_tick += period;
    std::this_thread::sleep_until(next_tick);

    const auto received = receiver.latest();
    if (!received.has_value()) {
      continue;  // nothing received yet (python: `if msg is None: continue`)
    }
    if (received->sequence != last_sequence) {
      ++fresh_packets;
      last_sequence = received->sequence;
    }
    const double now_s = std::chrono::duration<double>(Clock::now() - t0).count();
    const quest_vr::TeleopFrame frame = teleop.step(received->packet, now_s);
    ++ticks;

    // >>> IK solve goes here (consumes frame.right.pose / frame.left.pose). <<<

    if (Clock::now() >= next_status) {
      next_status += kStatusPeriod;
      logf(LogLevel::kInfo, "ticks={} fresh_packets/s={} seq={} R{} {} L{} {}", ticks,
           fresh_packets, received->sequence, frame.right.engaged ? "*" : " ",
           describePose(frame.right.pose), frame.left.engaged ? "*" : " ",
           describePose(frame.left.pose));
      fresh_packets = 0;
    }
  }
  logf(LogLevel::kInfo, "shutting down");
  receiver.stop();
  return EXIT_SUCCESS;
}
