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
//   4. q ← q + Δq (only if both arms' QPs succeeded)
//
// Everything per arm lives in ArmIkTask; the only thing the arms share inside
// an iteration is that scalar Σμ. With IkThreading::kThreadPerArm each arm
// runs on its own worker thread with its own MuJoCo data; the two workers meet
// at a spin barrier to exchange μ (and the QP success flags) every iteration,
// so the result is bit-for-bit the same as the single-thread solve.
//
// Same semantics as python: a solve needs a fresh target for every active arm
// since the last successful solve (`_pending`); the posture target is the
// configuration at construction; iterations already applied are kept if a
// later one fails. Not ported on purpose: the python fallback that re-solves
// *without joint limits* when the QP fails — box limits are always feasible.
//
// Threading contract: all public methods are called from one thread (the
// control loop). Workers only run inside solve(), which blocks until done.
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

#include "openarm_ik/arm_kinematics.hpp"
#include "openarm_ik/box_qp.hpp"
#include "openarm_ik/constants.hpp"
#include "openarm_ik/spin_barrier.hpp"
#include "openarm_ik/types.hpp"
#include "quest_vr/logging.hpp"

namespace openarm_ik {

enum class IkThreading {
  kSingleThread,   // both arms on the caller's thread
  kThreadPerArm,   // one worker thread (and one MuJoCo model/data) per arm
};

struct IkParams {
  double position_cost = kDefaultPositionCost;
  double orientation_cost = kDefaultOrientationCost;
  double task_gain = kDefaultTaskGain;
  double lm_damping = kDefaultLmDamping;
  double damping = kDefaultDamping;
  double posture_cost = kDefaultPostureCost;
  double limit_gain = kDefaultLimitGain;
  int max_iterations = kDefaultMaxIterations;
  // DAQP = the solver python uses on the robot; kActiveSet = self-written.
  QpBackend qp_backend = QpBackend::kDaqp;
  IkThreading threading = IkThreading::kSingleThread;
};

enum class IkStatus {
  kSolved,
  kNotReady,   // an active arm has no fresh target since the last solve
  kQpFailed,   // a QP failed; iterations before it were kept
};
const char* ikStatusName(IkStatus status);

// One arm's share of an IK iteration. Not thread-safe by itself: one thread
// at a time, and `kinematics` must not be touched by anyone else meanwhile.
class ArmIkTask {
 public:
  ArmIkTask(ArmKinematics& kinematics, ArmSide side, const IkParams& params, bool active);

  ArmSide side() const { return side_; }
  bool active() const { return active_; }
  void setTarget(const FramePose& target) { target_ = target; }
  void setPostureTarget(const JointVector& q) { posture_target_ = q; }

  // Phase 1: FrameTask terms at the current joints. Returns this arm's LM μ
  // (0 when the arm has no FrameTask).
  double prepare();
  // Phase 2: build and solve this arm's QP with the μ summed over all arms.
  // The step is kept, not applied. false (logged, throttled) if the QP failed.
  bool solveStep(double lm_mu_total);
  // Phase 3: q ← q + Δq.
  void applyStep();

 private:
  ArmKinematics& kinematics_;
  const ArmSide side_;
  const IkParams& params_;
  const bool active_;
  Twist cost_weights_;
  FramePose target_;
  JointVector posture_target_ = JointVector::Zero();
  Twist weighted_error_ = Twist::Zero();
  ArmJacobian weighted_jacobian_ = ArmJacobian::Zero();
  JointVector step_ = JointVector::Zero();
  quest_vr::LogEveryN qp_failure_log_{kHotLoopLogEvery};
};

class IkSolver {
 public:
  // Loads the model (twice with kThreadPerArm: one MuJoCo data per worker).
  // `active_sides` = arms with a FrameTask (python --mode); arms not listed
  // only feel the posture task and stay put. nullptr (logged) on bad input.
  static std::unique_ptr<IkSolver> create(const ArmKinematics::Config& model,
                                          const IkParams& params,
                                          std::span<const ArmSide> active_sides);
  ~IkSolver();
  IkSolver(const IkSolver&) = delete;
  IkSolver& operator=(const IkSolver&) = delete;

  // New TCP target (MuJoCo world frame). Rejects (false + log) inactive arms,
  // non-finite values and zero quaternions; the quaternion is normalised.
  bool setTarget(ArmSide side, const FramePose& target);
  bool isActive(ArmSide side) const { return active_.at(sideIndex(side)); }
  // True when every active arm got a target since the last successful solve.
  bool ready() const;

  // Overwrite the current joint positions (e.g. from /joint_states).
  bool syncJointPositions(ArmSide side, const JointVector& q);
  // Posture task target (defaults to the joints at construction).
  bool setPostureTarget(ArmSide side, const JointVector& q);

  IkStatus solve();
  JointVector jointPositions(ArmSide side) const;
  FramePose tcpPose(ArmSide side) const;  // FK at the current joints
  const IkParams& params() const { return params_; }

 private:
  IkSolver(const IkParams& params, std::array<bool, kNumArms> active);
  ArmKinematics& kinematicsFor(ArmSide side) const;
  bool solveSingleThread();
  bool solveThreaded();
  void workerLoop(ArmSide side);

  IkParams params_;
  std::array<bool, kNumArms> active_{};
  std::array<bool, kNumArms> pending_{};  // true = still waiting for a target
  // 1 kinematics shared by both tasks, or 1 per arm (kThreadPerArm).
  std::vector<std::unique_ptr<ArmKinematics>> kinematics_;
  std::array<std::unique_ptr<ArmIkTask>, kNumArms> tasks_{};

  // ── kThreadPerArm ─────────────────────────────────────────────────────────
  std::array<std::thread, kNumArms> workers_{};
  std::mutex job_mutex_;
  std::condition_variable job_cv_;
  std::uint64_t job_generation_ = 0;  // guarded by job_mutex_
  bool stop_ = false;                 // guarded by job_mutex_
  std::atomic<int> workers_done_{0};
  std::atomic<bool> job_failed_{false};
  SpinBarrier barrier_{static_cast<int>(kNumArms)};
  // Double-buffered by iteration parity: a fast worker can write iteration
  // k+1 while the other still reads iteration k (it cannot get further ahead
  // than that: the barrier of k+1 waits for it).
  static constexpr std::size_t kSlots = 2;
  std::array<std::array<double, kNumArms>, kSlots> mu_slots_{};
  std::array<std::array<bool, kNumArms>, kSlots> ok_slots_{};
};

}  // namespace openarm_ik
