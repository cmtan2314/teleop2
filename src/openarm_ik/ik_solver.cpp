#include "openarm_ik/ik_solver.hpp"

#include <cmath>

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

const char* threadingName(IkThreading threading) {
  return threading == IkThreading::kThreadPerArm ? "thread per arm" : "single thread";
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

// ── ArmIkTask ──────────────────────────────────────────────────────────────

ArmIkTask::ArmIkTask(ArmKinematics& kinematics, ArmSide side, const IkParams& params,
                     bool active)
    : kinematics_(kinematics), side_(side), params_(params), active_(active) {
  cost_weights_ << Eigen::Vector3d::Constant(params_.position_cost),
      Eigen::Vector3d::Constant(params_.orientation_cost);
  posture_target_ = kinematics_.jointPositions(side_);
}

double ArmIkTask::prepare() {
  if (!active_) {
    return 0.0;
  }
  const FramePose current = kinematics_.tcpPose(side_);
  const ArmJacobian body_jacobian = kinematics_.tcpJacobianLocal(side_);
  const Twist error = frameTaskError(target_, current);
  const ArmJacobian task_jacobian = frameTaskJacobian(target_, current, body_jacobian);
  weighted_error_ = cost_weights_.cwiseProduct(-params_.task_gain * error);
  weighted_jacobian_ = cost_weights_.asDiagonal() * task_jacobian;
  return params_.lm_damping * weighted_error_.squaredNorm();
}

bool ArmIkTask::solveStep(double lm_mu_total) {
  const double posture_weight_sq = params_.posture_cost * params_.posture_cost;
  const JointVector q = kinematics_.jointPositions(side_);
  JointMatrix hessian =
      (params_.damping + lm_mu_total + posture_weight_sq) * JointMatrix::Identity();
  JointVector gradient = posture_weight_sq * (q - posture_target_);
  if (active_) {
    hessian += weighted_jacobian_.transpose() * weighted_jacobian_;
    gradient -= weighted_jacobian_.transpose() * weighted_error_;
  }
  const ArmIndices& arm = kinematics_.indices(side_);
  const JointVector lower = params_.limit_gain * (arm.lower - q);
  const JointVector upper = params_.limit_gain * (arm.upper - q);

  const BoxQpResult qp = solveBoxQp(hessian, gradient, lower, upper, params_.qp_backend);
  if (qp.status != BoxQpStatus::kOptimal) {
    if (qp_failure_log_.shouldLog()) {
      logf(LogLevel::kError,
           "IK: {} arm QP failed ({} via {}, exitflag {}, {} iterations) — {} failure(s)",
           sideName(side_), boxQpStatusName(qp.status), qpBackendName(params_.qp_backend),
           qp.solver_exitflag, qp.iterations, qp_failure_log_.count());
    }
    return false;
  }
  step_ = qp.solution;
  return true;
}

void ArmIkTask::applyStep() {
  kinematics_.setJointPositions(side_, kinematics_.jointPositions(side_) + step_);
}

// ── IkSolver ───────────────────────────────────────────────────────────────

std::unique_ptr<IkSolver> IkSolver::create(const ArmKinematics::Config& model,
                                           const IkParams& params,
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
  // Private constructor → no make_unique.
  std::unique_ptr<IkSolver> solver(new IkSolver(params, active));

  const std::size_t num_models = params.threading == IkThreading::kThreadPerArm ? kNumArms : 1;
  for (std::size_t i = 0; i < num_models; ++i) {
    auto kinematics = ArmKinematics::create(model);
    if (kinematics == nullptr) {
      return nullptr;  // create() logged why
    }
    solver->kinematics_.push_back(std::move(kinematics));
  }
  for (const ArmSide side : kArmSides) {
    solver->tasks_.at(sideIndex(side)) = std::make_unique<ArmIkTask>(
        solver->kinematicsFor(side), side, solver->params_, active.at(sideIndex(side)));
  }
  if (params.threading == IkThreading::kThreadPerArm) {
    for (const ArmSide side : kArmSides) {
      solver->workers_.at(sideIndex(side)) =
          std::thread([raw = solver.get(), side] { raw->workerLoop(side); });
    }
  }
  logf(LogLevel::kInfo,
       "IkSolver: right {}, left {} | pos {} ori {} gain {} lm {} damping {} posture {} "
       "limit_gain {} iters {} qp {} | {}",
       active[sideIndex(ArmSide::kRight)] ? "active" : "posture-only",
       active[sideIndex(ArmSide::kLeft)] ? "active" : "posture-only", params.position_cost,
       params.orientation_cost, params.task_gain, params.lm_damping, params.damping,
       params.posture_cost, params.limit_gain, params.max_iterations,
       qpBackendName(params.qp_backend), threadingName(params.threading));
  return solver;
}

IkSolver::IkSolver(const IkParams& params, std::array<bool, kNumArms> active)
    : params_(params), active_(active), pending_(active) {}

IkSolver::~IkSolver() {
  {
    const std::lock_guard<std::mutex> lock(job_mutex_);
    stop_ = true;
  }
  job_cv_.notify_all();
  for (std::thread& worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

ArmKinematics& IkSolver::kinematicsFor(ArmSide side) const {
  return kinematics_.size() == 1 ? *kinematics_.front() : *kinematics_.at(sideIndex(side));
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
  FramePose normalized = target;
  normalized.orientation.normalize();
  tasks_.at(sideIndex(side))->setTarget(normalized);
  pending_.at(sideIndex(side)) = false;
  return true;
}

bool IkSolver::ready() const {
  for (const bool waiting : pending_) {
    if (waiting) {
      return false;
    }
  }
  return true;
}

bool IkSolver::syncJointPositions(ArmSide side, const JointVector& q) {
  // ArmKinematics rejects + logs non-finite values.
  return kinematicsFor(side).setJointPositions(side, q);
}

bool IkSolver::setPostureTarget(ArmSide side, const JointVector& q) {
  if (!q.allFinite()) {
    logf(LogLevel::kError, "IkSolver::setPostureTarget({}): non-finite values rejected",
         sideName(side));
    return false;
  }
  tasks_.at(sideIndex(side))->setPostureTarget(q);
  logf(LogLevel::kInfo, "IkSolver: {} posture target updated", sideName(side));
  return true;
}

JointVector IkSolver::jointPositions(ArmSide side) const {
  return kinematicsFor(side).jointPositions(side);
}

FramePose IkSolver::tcpPose(ArmSide side) const { return kinematicsFor(side).tcpPose(side); }

IkStatus IkSolver::solve() {
  if (!ready()) {
    // Caller should check ready() first (the python node does); not an error.
    logf(LogLevel::kDebug, "IkSolver::solve: waiting for targets");
    return IkStatus::kNotReady;
  }
  const bool ok = params_.threading == IkThreading::kThreadPerArm ? solveThreaded()
                                                                   : solveSingleThread();
  if (!ok) {
    return IkStatus::kQpFailed;  // pending_ kept, like python returning None
  }
  pending_ = active_;
  return IkStatus::kSolved;
}

bool IkSolver::solveSingleThread() {
  ArmIkTask& right = *tasks_[sideIndex(ArmSide::kRight)];
  ArmIkTask& left = *tasks_[sideIndex(ArmSide::kLeft)];
  for (int i = 0; i < params_.max_iterations; ++i) {
    // Both arms' terms at the same configuration, then the shared μ.
    const double mu_total = right.prepare() + left.prepare();
    const bool right_ok = right.solveStep(mu_total);
    const bool left_ok = left.solveStep(mu_total);
    if (!right_ok || !left_ok) {
      return false;  // python: the whole QP failed → nothing applied this iteration
    }
    right.applyStep();
    left.applyStep();
  }
  return true;
}

bool IkSolver::solveThreaded() {
  job_failed_.store(false, std::memory_order_relaxed);
  workers_done_.store(0, std::memory_order_relaxed);
  {
    const std::lock_guard<std::mutex> lock(job_mutex_);
    ++job_generation_;
  }
  job_cv_.notify_all();
  // The solve takes tens of µs: spin rather than sleep so the result is
  // picked up as soon as the second worker finishes.
  while (workers_done_.load(std::memory_order_acquire) < static_cast<int>(kNumArms)) {
    std::this_thread::yield();
  }
  return !job_failed_.load(std::memory_order_relaxed);
}

void IkSolver::workerLoop(ArmSide side) {
  const std::size_t k = sideIndex(side);
  ArmIkTask& task = *tasks_.at(k);
  std::uint64_t seen_generation = 0;
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(job_mutex_);
      job_cv_.wait(lock, [&] { return stop_ || job_generation_ != seen_generation; });
      if (stop_) {
        return;
      }
      seen_generation = job_generation_;
    }
    for (int i = 0; i < params_.max_iterations; ++i) {
      const std::size_t slot = static_cast<std::size_t>(i) % kSlots;
      mu_slots_[slot][k] = task.prepare();
      barrier_.arriveAndWait();  // both μ written
      // Same summation order as solveSingleThread(): right + left.
      const double mu_total =
          mu_slots_[slot][sideIndex(ArmSide::kRight)] + mu_slots_[slot][sideIndex(ArmSide::kLeft)];
      ok_slots_[slot][k] = task.solveStep(mu_total);
      barrier_.arriveAndWait();  // both QP results known
      if (!ok_slots_[slot][sideIndex(ArmSide::kRight)] ||
          !ok_slots_[slot][sideIndex(ArmSide::kLeft)]) {
        job_failed_.store(true, std::memory_order_relaxed);
        break;  // both workers see the same flags → both stop here
      }
      task.applyStep();
    }
    workers_done_.fetch_add(1, std::memory_order_acq_rel);
  }
}

}  // namespace openarm_ik
