// Constants for the C++ port of the mink-based OpenArm IK (openarm_control 0.1.0).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace openarm_ik {

inline constexpr std::size_t kNumArmJoints = 7;
// se(3) twist dimension, ordered [vx, vy, vz, wx, wy, wz] like mink.
inline constexpr std::size_t kTwistDim = 6;
inline constexpr std::size_t kNumArms = 2;

// MJCF names (openarm_mujoco/v1/openarm_bimanual.xml). Joint i (1-based) of a
// side is "openarm_<side>_joint<i>", the same names ROS 2 uses on /joint_states.
inline constexpr std::string_view kJointNamePrefix = "openarm_";
inline constexpr std::string_view kJointNameInfix = "_joint";
inline constexpr std::string_view kDefaultRightTcpBody = "openarm_right_tcp";
inline constexpr std::string_view kDefaultLeftTcpBody = "openarm_left_tcp";
inline constexpr std::string_view kDefaultKeyframe = "home";

// Size of the buffer MuJoCo writes XML load errors into.
inline constexpr std::size_t kMjErrorBufferSize = 1024;

// A log site inside the solve loop (500 Hz × 10 iterations) prints its 1st and
// then every Nth occurrence.
inline constexpr std::uint64_t kHotLoopLogEvery = 1000;

// ── Step 2: SE(3) maths (mink/lie/_lie_ops_c.c) ──────────────────────────────
// Below this (angle, or angle² where mink compares the square) the closed-form
// expressions divide by ~0, so mink switches to its Taylor series.
inline constexpr double kLieSmallAngleEpsilon = 1e-10;

// ── Step 3: box-constrained QP ───────────────────────────────────────────────
// Each active-set iteration adds or releases one bound; 7 variables need far
// fewer than this unless the problem is degenerate.
inline constexpr int kBoxQpMaxIterations = 100;
// A Newton step whose largest component is below this (rad) means x is
// optimal on the current free subspace.
inline constexpr double kBoxQpStepTolerance = 1e-12;
// Bound multipliers more negative than this mean the bound should be released.
inline constexpr double kBoxQpDualTolerance = 1e-12;

// ── Step 4: IK defaults = config/dataflow_bridge_ros2_vr.yaml `ik` node args ─
// (--damping 0.1 --posture-cost 0.01 --lm-damping 0.01 --max-iters 10) plus
// openarm_control 0.1.0 / mink 1.1 defaults for the rest.
inline constexpr double kDefaultPositionCost = 1.0;     // --pos-cost
inline constexpr double kDefaultOrientationCost = 1.0;  // --ori-cost
inline constexpr double kDefaultTaskGain = 1.0;         // mink FrameTask gain (dead-beat)
inline constexpr double kDefaultLmDamping = 0.01;       // --lm-damping
inline constexpr double kDefaultDamping = 0.1;          // --damping (Tikhonov on all DOFs)
inline constexpr double kDefaultPostureCost = 0.01;     // --posture-cost
inline constexpr double kDefaultLimitGain = 0.95;       // mink ConfigurationLimit gain
inline constexpr int kDefaultMaxIterations = 10;        // --max-iters

}  // namespace openarm_ik
