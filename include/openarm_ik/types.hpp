// Plain data types shared by every stage of the IK port. No MuJoCo here, so the
// SE(3) maths and the QP solver can be built and tested without the model.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstddef>

#include "openarm_ik/constants.hpp"

namespace openarm_ik {

enum class ArmSide : std::size_t { kRight = 0, kLeft = 1 };
inline constexpr std::array<ArmSide, kNumArms> kArmSides = {ArmSide::kRight, ArmSide::kLeft};
const char* sideName(ArmSide side);
inline std::size_t sideIndex(ArmSide side) { return static_cast<std::size_t>(side); }

using JointVector = Eigen::Matrix<double, kNumArmJoints, 1>;
using JointMatrix = Eigen::Matrix<double, kNumArmJoints, kNumArmJoints>;
// se(3) twist [vx, vy, vz, wx, wy, wz] (linear first, like mink).
using Twist = Eigen::Matrix<double, kTwistDim, 1>;
using TwistMatrix = Eigen::Matrix<double, kTwistDim, kTwistDim>;
// Rows: twist components, columns: the 7 arm joints.
using ArmJacobian = Eigen::Matrix<double, kTwistDim, kNumArmJoints>;

// Rigid transform frame → world (mink's SE3 / wxyz_xyz). Orientation must be a
// unit quaternion for the SE(3) functions to be valid.
struct FramePose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

}  // namespace openarm_ik
