// Step 1 of the IK port: MuJoCo-backed kinematics for the two OpenArm arms.
//
// Owns one mjModel/mjData loaded from the same MJCF the python IK uses
// (openarm_mujoco/v1/scene.xml). Joint/body indices are resolved by name once
// at load time — never hard-coded — so the qpos layout of the XML
// (left arm first, then right) is irrelevant to callers.
//
// Frame conventions (identical to mink):
//   * poses are in the MuJoCo world frame (base link at z = 0.34 m),
//   * the TCP Jacobian is the *body* (local-frame) Jacobian, rows [v; w].
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <memory>
#include <string>

#include "openarm_ik/constants.hpp"
#include "openarm_ik/types.hpp"
#include "quest_vr/logging.hpp"

namespace openarm_ik {

// Where one arm lives inside the MuJoCo model.
struct ArmIndices {
  std::array<int, kNumArmJoints> qpos{};  // index into mjData::qpos for joint1..joint7
  std::array<int, kNumArmJoints> dof{};   // column in a 6×nv Jacobian for joint1..joint7
  JointVector lower = JointVector::Zero();  // joint limits (rad), from jnt_range
  JointVector upper = JointVector::Zero();
  int tcp_body = -1;
};

class ArmKinematics {
 public:
  struct Config {
    std::string xml_path;
    std::string right_tcp_body{kDefaultRightTcpBody};
    std::string left_tcp_body{kDefaultLeftTcpBody};
    // Initial configuration. Empty, or not present in the model → qpos0 (all zeros).
    std::string keyframe{kDefaultKeyframe};
  };

  // Loads the model and resolves all indices. Returns nullptr (after logging
  // the reason) if the XML fails to load or any joint/body is missing.
  static std::unique_ptr<ArmKinematics> create(const Config& config);

  ~ArmKinematics();
  ArmKinematics(const ArmKinematics&) = delete;
  ArmKinematics& operator=(const ArmKinematics&) = delete;

  const ArmIndices& indices(ArmSide side) const;

  JointVector jointPositions(ArmSide side) const;
  // Rejects (returns false, logs) non-finite values. Values outside the joint
  // limits are accepted but logged: the real robot can legitimately report
  // slightly out-of-range angles and the IK must start from where it really is.
  bool setJointPositions(ArmSide side, const JointVector& q);

  // TCP pose / body Jacobian at the current joint positions. Kinematics are
  // recomputed lazily (mj_kinematics + mj_comPos) after any setJointPositions().
  FramePose tcpPose(ArmSide side) const;
  ArmJacobian tcpJacobianLocal(ArmSide side) const;

  const mjModel& model() const { return *model_; }

 private:
  ArmKinematics(mjModel* model, mjData* data);
  bool resolveArm(ArmSide side, const std::string& tcp_body);
  void applyKeyframe(const std::string& keyframe);
  void ensureForward() const;

  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  std::array<ArmIndices, kNumArms> arms_{};
  mutable bool kinematics_dirty_ = true;
  // setJointPositions() runs every IK iteration; a robot parked just outside a
  // limit would otherwise log the same warning thousands of times per second.
  quest_vr::LogEveryN out_of_limits_log_{kHotLoopLogEvery};
};

}  // namespace openarm_ik
