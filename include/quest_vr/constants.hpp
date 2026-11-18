// C++ port of vendor/dora-openarm-vr (Meta Quest UDP receiver + pose smoothing).
// All tunable values live here so there is a single source of truth.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace quest_vr {

// ── UDP receiver (udp_receiver.py) ───────────────────────────────────────────
inline constexpr std::string_view kDefaultHost = "0.0.0.0";
inline constexpr std::uint16_t kDefaultPort = 5006;
inline constexpr std::size_t kDefaultRecvBufferBytes = 4096;
// Poll timeout so the receiver thread notices stop() promptly (python: settimeout(1.0)).
inline constexpr int kRecvPollTimeoutMs = 1000;
// Delay before re-opening the socket after an OS error (python: time.sleep(1.0)).
inline constexpr int kSocketRetryDelayMs = 1000;
// Max number of packet-arrival timestamps kept between drains (python: deque(maxlen=512)).
inline constexpr std::size_t kArrivalTimestampCapacity = 512;
// Log the 1st and then every Nth bad packet, so a misbehaving sender cannot flood the log.
inline constexpr std::uint64_t kBadPacketLogEvery = 100;

// ── Dead-man grip (quest_receiver.py main loop) ─────────────────────────────
inline constexpr double kGripEngageThreshold = 0.5;  // grip axis 0..1
inline constexpr double kGripHoldWindowS = 5.0;      // GRIP_HOLD_SECONDS

// ── One Euro smoother parameters used by the receiver node ─────────────────
inline constexpr double kSmootherMinCutoffHz = 2.0;
inline constexpr double kSmootherBeta = 0.04;
inline constexpr double kSmootherDerivativeCutoffHz = 1.5;
// Above this quaternion dot product, slerp falls back to normalized lerp.
inline constexpr double kSlerpLinearThreshold = 0.9995;

// ── Frame alignment (quest_receiver.py "edit here to tune") ────────────────
// Rotation from the right-handed VR frame (x right, y up, z back) to the robot
// frame (x forward, y left, z up). Row-major 3x3.
inline constexpr std::array<double, 9> kVrToRobotRotationRowMajor = {
    0.0,  0.0, -1.0,  //
    -1.0, 0.0, 0.0,   //
    0.0,  1.0, 0.0,
};
// Robot-frame point (MuJoCo world, metres) the reference (headset) pose maps to.
inline constexpr std::array<double, 3> kFrameOffsetNeckM = {0.1, 0.0, 1.2};
// R_FIX: extra right-multiplied rotation about z aligning controller axes to the TCP.
inline constexpr double kControllerFixYawDeg = 90.0;

// Where the captured reference pose is written for inspection, relative to $HOME.
inline constexpr std::string_view kReferenceCacheRelPath =
    ".cache/dora_openarm_vr/reference_pose.json";

}  // namespace quest_vr
