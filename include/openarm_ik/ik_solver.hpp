// Step 4 of the IK port: the differential IK loop of openarm_control 0.1.0
// (`_IKSolver`, kinematics.py) on top of steps 1–3.
//
// Per solve(): max_iterations times
//   1. FK + body Jacobian of each active TCP                      (step 1)
//   2. FrameTask error e = log(T_tcp⁻¹ · T_target), J_task = −Jlog·J_body (step 2)
//   3. Build the mink QP per arm and solve it                     (step 3)
//        H = (damping + Σμ + w_p²)·I + (W·J_task)ᵀ(W·J_task)
//        c = w_p²·(q − q_posture) − (W·J_task)ᵀ·W·(−gain·e)
//        gain_lim·(q_min − q) ≤ Δq ≤ gain_lim·(q_max − q)
//      with W = diag(pos_cost×3, ori_cost×3), w_p = posture cost and
//      μ = lm_damping·‖W·gain·e‖² summed over the active FrameTasks — mink adds
//      every task's μ to *all* DOFs, so each arm's QP sees both arms' μ.
//   4. q ← q + Δq
//
// Same semantics as python: a solve needs a fresh target for every active arm
// since the last successful solve (`_pending`); the posture target is the
// configuration at construction; the iterations already applied are kept if a
// later one fails. Not ported on purpose: the python fallback that re-solves
// *without joint limits* when the QP fails — box limits are always feasible,
// so that path only hides solver bugs.
#pragma once

#include <array>
#include <memory>
#include <span>

#include "openarm_ik/arm_kinematics.hpp"
#include "openarm_ik/constants.hpp"
#include "openarm_ik/types.hpp"
#include "quest_vr/logging.hpp"

namespace openarm_ik {

struct IkParams {
  double position_cost = kDefaultPositionCost;
  double orientation_cost = kDefaultOrientationCost;
  double task_gain = kDefaultTaskGain;
  double lm_damping = kDefaultLmDamping;
  double damping = kDefaultDamping;
  double posture_cost = kDefaultPostureCost;
  double limit_gain = kDefaultLimitGain;
  int max_iterations = kDefaultMaxIterations;
};

enum class IkStatus {
  kSolved,
  kNotReady,   // an active arm has no fresh target since the last solve
  kQpFailed,   // the box QP failed; iterations before it were kept
};
const char* ikStatusName(IkStatus status);

class IkSolver {
 public:
  // `kinematics` must outlive the solver; the solver reads and writes its joint
  // positions. `active_sides` = arms with a FrameTask (python --mode); arms not
  // listed only feel the posture task and stay where they are. Returns nullptr
  // (after logging why) on invalid parameters.
  static std::unique_ptr<IkSolver> create(ArmKinematics& kinematics, const IkParams& params,
                                          std::span<const ArmSide> active_sides);

  // New TCP target (world frame, like FramePose from step 1). Rejects (false +
  // log) inactive arms, non-finite values and zero quaternions; the quaternion
  // is normalised.
  bool setTarget(ArmSide side, const FramePose& target);
  bool isActive(ArmSide side) const { return active_.at(sideIndex(side)); }
  // True when every active arm got a target since the last successful solve.
  bool ready() const;

  // Overwrite the current joint positions (e.g. from /joint_states) — the IK
  // then continues from where the robot really is.
  bool syncJointPositions(ArmSide side, const JointVector& q);
  // Posture task target (defaults to the joints at construction).
  bool setPostureTarget(ArmSide side, const JointVector& q);

  IkStatus solve();
  JointVector jointPositions(ArmSide side) const { return kinematics_.jointPositions(side); }
  const IkParams& params() const { return params_; }

 private:
  IkSolver(ArmKinematics& kinematics, const IkParams& params,
           std::array<bool, kNumArms> active);
  bool iterate();  // one QP step for both arms; false if a QP failed

  ArmKinematics& kinematics_;
  IkParams params_;
  std::array<bool, kNumArms> active_{};
  std::array<FramePose, kNumArms> target_{};
  std::array<bool, kNumArms> pending_{};  // true = still waiting for a target
  std::array<JointVector, kNumArms> posture_target_{};
  quest_vr::LogEveryN qp_failure_log_{kHotLoopLogEvery};
};

}  // namespace openarm_ik
