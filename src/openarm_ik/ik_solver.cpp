#include "openarm_ik/ik_solver.hpp"

#include <cmath>

#include "openarm_ik/box_qp.hpp"
#include "openarm_ik/se3.hpp"

namespace openarm_ik {
namespace {

using quest_vr::LogLevel;
using quest_vr::logf;

// A target quaternion shorter than this cannot be normalised into a rotation.
constexpr double kMinQuaternionNorm = 1e-6;

bool validParams(const IkParams& p) {
  // mink rejects the same values (Task / ConfigurationLimit constructors).
  if (!(p.position_cost >= 0.0) || !(p.orientation_cost >= 0.0) || !(p.posture_cost >= 0.0)) {
    logf(LogLevel::kError, "IkSolver: costs must be >= 0 (pos {}, ori {}, posture {})",
         p.position_cost, p.orientation_cost, p.posture_cost);
    return false;
  }
  if (!(p.task_gain >= 0.0 && p.task_gain <= 1.0)) {
    logf(LogLevel::kError, "IkSolver: task_gain {} outside [0, 1]", p.task_gain);
    return false;
  }
  if (!(p.limit_gain > 0.0 && p.limit_gain <= 1.0)) {
    logf(LogLevel::kError, "IkSolver: limit_gain {} outside (0, 1]", p.limit_gain);
    return false;
  }
  if (!(p.lm_damping >= 0.0)) {
    logf(LogLevel::kError, "IkSolver: lm_damping {} must be >= 0", p.lm_damping);
    return false;
  }
  // damping > 0 is what makes every per-arm Hessian positive definite.
  if (!(p.damping > 0.0) || !std::isfinite(p.damping)) {
    logf(LogLevel::kError, "IkSolver: damping {} must be > 0 and finite", p.damping);
    return false;
  }
  if (p.max_iterations <= 0) {
    logf(LogLevel::kError, "IkSolver: max_iterations {} must be > 0", p.max_iterations);
    return false;
  }
  return true;
}

}  // namespace

const char* ikStatusName(IkStatus status) {
  switch (status) {
    case IkStatus::kSolved:
      return "solved";
    case IkStatus::kNotReady:
      return "not ready";
    case IkStatus::kQpFailed:
      return "QP failed";
  }
  return "unknown";
}

std::unique_ptr<IkSolver> IkSolver::create(ArmKinematics& kinematics, const IkParams& params,
                                           std::span<const ArmSide> active_sides) {
  if (!validParams(params)) {
    return nullptr;  // validParams() logged which one
  }
  if (active_sides.empty()) {
    logf(LogLevel::kError, "IkSolver: no active arm requested");
    return nullptr;
  }
  std::array<bool, kNumArms> active{};
  for (const ArmSide side : active_sides) {
    if (sideIndex(side) >= kNumArms) {
      logf(LogLevel::kError, "IkSolver: invalid arm index {}", sideIndex(side));
      return nullptr;
    }
    active.at(sideIndex(side)) = true;
  }
  logf(LogLevel::kInfo,
       "IkSolver: right {}, left {} | pos {} ori {} gain {} lm {} damping {} posture {} "
       "limit_gain {} iters {}",
       active[sideIndex(ArmSide::kRight)] ? "active" : "posture-only",
       active[sideIndex(ArmSide::kLeft)] ? "active" : "posture-only", params.position_cost,
       params.orientation_cost, params.task_gain, params.lm_damping, params.damping,
       params.posture_cost, params.limit_gain, params.max_iterations);
  // Private constructor → no make_unique.
  return std::unique_ptr<IkSolver>(new IkSolver(kinematics, params, active));
}

IkSolver::IkSolver(ArmKinematics& kinematics, const IkParams& params,
                   std::array<bool, kNumArms> active)
    : kinematics_(kinematics), params_(params), active_(active), pending_(active) {
  for (const ArmSide side : kArmSides) {
    posture_target_.at(sideIndex(side)) = kinematics_.jointPositions(side);
  }
}

bool IkSolver::setTarget(ArmSide side, const FramePose& target) {
  if (!isActive(side)) {
    logf(LogLevel::kWarn, "IkSolver::setTarget: {} arm is not active, target ignored",
         sideName(side));
    return false;
  }
  if (!target.position.allFinite() || !target.orientation.coeffs().allFinite()) {
    logf(LogLevel::kError, "IkSolver::setTarget({}): non-finite target rejected",
         sideName(side));
    return false;
  }
  const double norm = target.orientation.norm();
  if (norm < kMinQuaternionNorm) {
    logf(LogLevel::kError, "IkSolver::setTarget({}): quaternion norm {} too small, rejected",
         sideName(side), norm);
    return false;
  }
  FramePose& stored = target_.at(sideIndex(side));
  stored.position = target.position;
  stored.orientation = target.orientation.normalized();
  pending_.at(sideIndex(side)) = false;
  return true;
}

bool IkSolver::ready() const {
  for (const ArmSide side : kArmSides) {
    if (pending_.at(sideIndex(side))) {
      return false;
    }
  }
  return true;
}

bool IkSolver::syncJointPositions(ArmSide side, const JointVector& q) {
  // ArmKinematics rejects + logs non-finite values.
  return kinematics_.setJointPositions(side, q);
}

bool IkSolver::setPostureTarget(ArmSide side, const JointVector& q) {
  if (!q.allFinite()) {
    logf(LogLevel::kError, "IkSolver::setPostureTarget({}): non-finite values rejected",
         sideName(side));
    return false;
  }
  posture_target_.at(sideIndex(side)) = q;
  logf(LogLevel::kInfo, "IkSolver: {} posture target updated", sideName(side));
  return true;
}

IkStatus IkSolver::solve() {
  if (!ready()) {
    // Caller should check ready() first (the python node does); not an error.
    logf(LogLevel::kDebug, "IkSolver::solve: waiting for targets");
    return IkStatus::kNotReady;
  }
  for (int i = 0; i < params_.max_iterations; ++i) {
    if (!iterate()) {
      return IkStatus::kQpFailed;  // pending_ kept, like python returning None
    }
  }
  pending_ = active_;
  return IkStatus::kSolved;
}

bool IkSolver::iterate() {
  Twist cost_weights;
  cost_weights << Eigen::Vector3d::Constant(params_.position_cost),
      Eigen::Vector3d::Constant(params_.orientation_cost);

  // FrameTask terms, all evaluated at the same configuration (before any update).
  std::array<ArmJacobian, kNumArms> weighted_jacobian{};
  std::array<Twist, kNumArms> weighted_error{};
  double lm_mu_total = 0.0;
  for (const ArmSide side : kArmSides) {
    const std::size_t k = sideIndex(side);
    if (!active_[k]) {
      continue;
    }
    const FramePose current = kinematics_.tcpPose(side);
    const ArmJacobian body_jacobian = kinematics_.tcpJacobianLocal(side);
    const Twist error = frameTaskError(target_[k], current);
    const ArmJacobian task_jacobian = frameTaskJacobian(target_[k], current, body_jacobian);
    weighted_error[k] = cost_weights.cwiseProduct(-params_.task_gain * error);
    weighted_jacobian[k] = cost_weights.asDiagonal() * task_jacobian;
    lm_mu_total += params_.lm_damping * weighted_error[k].squaredNorm();
  }

  const double posture_weight_sq = params_.posture_cost * params_.posture_cost;
  const double diagonal = params_.damping + lm_mu_total + posture_weight_sq;
  std::array<JointVector, kNumArms> delta{};
  for (const ArmSide side : kArmSides) {
    const std::size_t k = sideIndex(side);
    const JointVector q = kinematics_.jointPositions(side);
    JointMatrix hessian = diagonal * JointMatrix::Identity();
    JointVector gradient = posture_weight_sq * (q - posture_target_[k]);
    if (active_[k]) {
      hessian += weighted_jacobian[k].transpose() * weighted_jacobian[k];
      gradient -= weighted_jacobian[k].transpose() * weighted_error[k];
    }
    const ArmIndices& arm = kinematics_.indices(side);
    const JointVector lower = params_.limit_gain * (arm.lower - q);
    const JointVector upper = params_.limit_gain * (arm.upper - q);

    const BoxQpResult qp = solveBoxQp(hessian, gradient, lower, upper);
    if (qp.status != BoxQpStatus::kOptimal) {
      if (qp_failure_log_.shouldLog()) {
        logf(LogLevel::kError, "IkSolver: {} arm QP failed ({}, {} iterations) — {} failure(s)",
             sideName(side), boxQpStatusName(qp.status), qp.iterations,
             qp_failure_log_.count());
      }
      return false;
    }
    delta[k] = qp.solution;
  }

  for (const ArmSide side : kArmSides) {
    const std::size_t k = sideIndex(side);
    kinematics_.setJointPositions(side, kinematics_.jointPositions(side) + delta[k]);
  }
  return true;
}

}  // namespace openarm_ik
