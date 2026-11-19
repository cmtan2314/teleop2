// Constants of the ROS 2 output stage — port of src/dora_ros2_bridge/main.py
// (the dora-to-ros2 node). Values are copied from main.py so behaviour matches.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace teleop_bridge {

inline constexpr std::size_t kNumHandJoints = 6;
inline constexpr std::size_t kNumThumbJoints = 2;  // hand[0:2] follows the thumbstick
inline constexpr std::size_t kNumButtons = 4;      // [a, b, x, y], /vr_buttons order

// --mode real safety ramp (get_safe_position).
inline constexpr double kArmMaxStepRad = 0.02;        // MAX_STEP per arm command
inline constexpr double kArmSyncThresholdRad = 0.1;   // SYNC_THRESHOLD: resync to feedback

// Revo2 hand ramp: each trigger / thumbstick event moves the targets at most this much.
inline constexpr double kHandMaxStepRad = 0.03;  // HAND_MAX_STEP
// The trigger reports ~the square of the lever angle; grip = raw^kGripLinearizeExponent.
inline constexpr double kGripLinearizeExponent = 0.5;
inline constexpr double kTriggerMin = 0.0;
inline constexpr double kTriggerMax = 1.0;
inline constexpr double kStickMin = -1.0;
inline constexpr double kStickMax = 1.0;

// HAND_CLOSED = preset "box", used when no --grasp-presets file is given.
inline constexpr std::string_view kDefaultPresetName = "box";
inline constexpr std::array<double, kNumHandJoints> kDefaultHandClosedRad = {1.57, 1.03, 0.4,
                                                                            0.5,  0.6,  0.7};

// Joint name pieces (/joint_states, controller joint lists).
inline constexpr std::array<std::string_view, kNumHandJoints> kHandFingerNames = {
    "thumb_metacarpal", "thumb_proximal", "index_proximal",
    "middle_proximal",  "ring_proximal",  "pinky_proximal"};
// --mode sim only: distal joints mirror hand[1:6] (thumb_proximal .. pinky_proximal).
inline constexpr std::array<std::string_view, kNumHandJoints - 1> kHandDistalNames = {
    "thumb_distal", "index_distal", "middle_distal", "ring_distal", "pinky_distal"};

// Heartbeat: one status line every N right-arm commands (main.py LOG_EVERY).
inline constexpr std::uint64_t kHeartbeatEvery = 200;

}  // namespace teleop_bridge
