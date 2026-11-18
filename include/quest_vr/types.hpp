#pragma once

#include <Eigen/Geometry>
#include <array>
#include <optional>

namespace quest_vr {

// Validity flags sent by the headset ("v", "vl", "vr").
enum class Validity : int {
  kOk = 0,       // normal processing
  kStale = 1,    // headset re-sends last good pose; smoothed normally
  kInvalid = 2,  // no pose output; smoother reset so re-entry is jump-free
};

const char* validityName(Validity validity);

// A pose as sent by Unity: left-handed world, metres, scalar-last quaternion.
struct UnityPose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
};

// A right-handed pose in the robot (MuJoCo world) frame.
struct Pose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

// Number of elements in the flat pose layout used by the dataflow.
inline constexpr std::size_t kFlatPoseSize = 7;

// Flat layout used by the Dora dataflow: [px, py, pz, qw, qx, qy, qz].
std::array<double, kFlatPoseSize> toFlatPose(const Pose& pose);

}  // namespace quest_vr
