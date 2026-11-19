#include "openarm_ik/arm_kinematics.hpp"

#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <vector>

#include "quest_vr/logging.hpp"

namespace openarm_ik {
namespace {

using quest_vr::LogLevel;
using quest_vr::logf;

constexpr int kRotationDim = 3;
constexpr int kQuaternionDim = 4;  // MuJoCo xquat layout [w, x, y, z]

std::string jointName(ArmSide side, std::size_t joint_index) {
  // joint_index is 0-based; MJCF/ROS names are 1-based (joint1..joint7).
  return std::format("{}{}{}{}", kJointNamePrefix, sideName(side), kJointNameInfix,
                     joint_index + 1);
}

}  // namespace

const char* sideName(ArmSide side) {
  switch (side) {
    case ArmSide::kRight:
      return "right";
    case ArmSide::kLeft:
      return "left";
  }
  return "unknown";
}

std::unique_ptr<ArmKinematics> ArmKinematics::create(const Config& config) {
  if (config.xml_path.empty()) {
    logf(LogLevel::kError, "ArmKinematics: empty MJCF path");
    return nullptr;
  }
  std::array<char, kMjErrorBufferSize> error{};
  mjModel* model =
      mj_loadXML(config.xml_path.c_str(), nullptr, error.data(), static_cast<int>(error.size()));
  if (model == nullptr) {
    logf(LogLevel::kError, "ArmKinematics: failed to load '{}': {}", config.xml_path,
         error.data());
    return nullptr;
  }
  mjData* data = mj_makeData(model);
  if (data == nullptr) {
    logf(LogLevel::kError, "ArmKinematics: mj_makeData failed for '{}'", config.xml_path);
    mj_deleteModel(model);
    return nullptr;
  }
  // Private constructor → no make_unique. From here the object owns model/data.
  std::unique_ptr<ArmKinematics> kinematics(new ArmKinematics(model, data));

  if (!kinematics->resolveArm(ArmSide::kRight, config.right_tcp_body) ||
      !kinematics->resolveArm(ArmSide::kLeft, config.left_tcp_body)) {
    return nullptr;  // resolveArm() logged which name is missing
  }
  kinematics->applyKeyframe(config.keyframe);
  logf(LogLevel::kInfo, "ArmKinematics: loaded '{}' (nq={}, nv={})", config.xml_path, model->nq,
       model->nv);
  return kinematics;
}

ArmKinematics::ArmKinematics(mjModel* model, mjData* data) : model_(model), data_(data) {}

ArmKinematics::~ArmKinematics() {
  if (data_ != nullptr) {
    mj_deleteData(data_);
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
  }
}

bool ArmKinematics::resolveArm(ArmSide side, const std::string& tcp_body) {
  ArmIndices& arm = arms_.at(sideIndex(side));
  for (std::size_t i = 0; i < kNumArmJoints; ++i) {
    const std::string name = jointName(side, i);
    const int joint_id = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
    if (joint_id < 0) {
      logf(LogLevel::kError, "ArmKinematics: joint '{}' not found in model", name);
      return false;
    }
    if (model_->jnt_type[joint_id] != mjJNT_HINGE) {
      logf(LogLevel::kError, "ArmKinematics: joint '{}' is not a hinge (type {})", name,
           model_->jnt_type[joint_id]);
      return false;
    }
    arm.qpos.at(i) = model_->jnt_qposadr[joint_id];
    arm.dof.at(i) = model_->jnt_dofadr[joint_id];
    if (model_->jnt_limited[joint_id] != 0) {
      arm.lower(static_cast<Eigen::Index>(i)) = model_->jnt_range[2 * joint_id];
      arm.upper(static_cast<Eigen::Index>(i)) = model_->jnt_range[2 * joint_id + 1];
    } else {
      logf(LogLevel::kWarn, "ArmKinematics: joint '{}' has no limits in the model", name);
      arm.lower(static_cast<Eigen::Index>(i)) = -std::numeric_limits<double>::infinity();
      arm.upper(static_cast<Eigen::Index>(i)) = std::numeric_limits<double>::infinity();
    }
  }
  arm.tcp_body = mj_name2id(model_, mjOBJ_BODY, tcp_body.c_str());
  if (arm.tcp_body < 0) {
    logf(LogLevel::kError, "ArmKinematics: TCP body '{}' not found in model", tcp_body);
    return false;
  }
  logf(LogLevel::kInfo, "ArmKinematics: {} arm qpos[{}..{}], TCP body '{}' (id {})",
       sideName(side), arm.qpos.front(), arm.qpos.back(), tcp_body, arm.tcp_body);
  return true;
}

void ArmKinematics::applyKeyframe(const std::string& keyframe) {
  kinematics_dirty_ = true;
  if (keyframe.empty()) {
    logf(LogLevel::kInfo, "ArmKinematics: no keyframe requested, starting from qpos0");
    mj_resetData(model_, data_);
    return;
  }
  const int key_id = mj_name2id(model_, mjOBJ_KEY, keyframe.c_str());
  if (key_id < 0) {
    // Same fallback as python (setup_from_args), but this start pose is the
    // straight-arm singularity — callers should overwrite it with the real
    // robot's joint positions before solving.
    logf(LogLevel::kWarn,
         "ArmKinematics: keyframe '{}' not found, starting from qpos0 (all joints 0)", keyframe);
    mj_resetData(model_, data_);
    return;
  }
  mj_resetDataKeyframe(model_, data_, key_id);
  logf(LogLevel::kInfo, "ArmKinematics: initial pose from keyframe '{}'", keyframe);
}

const ArmIndices& ArmKinematics::indices(ArmSide side) const { return arms_.at(sideIndex(side)); }

JointVector ArmKinematics::jointPositions(ArmSide side) const {
  const ArmIndices& arm = indices(side);
  JointVector q;
  for (std::size_t i = 0; i < kNumArmJoints; ++i) {
    q(static_cast<Eigen::Index>(i)) = data_->qpos[arm.qpos.at(i)];
  }
  return q;
}

bool ArmKinematics::setJointPositions(ArmSide side, const JointVector& q) {
  if (!q.allFinite()) {
    logf(LogLevel::kError, "setJointPositions({}): rejecting non-finite joint values",
         sideName(side));
    return false;
  }
  const ArmIndices& arm = indices(side);
  // Same tolerance as mink Configuration.check_limits(): the IK walks right
  // up to a limit and rounding can land 1e-17 rad past it.
  const bool within_limits = (q.array() >= arm.lower.array() - kJointLimitToleranceRad).all() &&
                             (q.array() <= arm.upper.array() + kJointLimitToleranceRad).all();
  if (!within_limits && out_of_limits_log_.shouldLog()) {
    logf(LogLevel::kWarn,
         "setJointPositions({}): joint values outside model limits (accepted, {} time(s) so far)",
         sideName(side), out_of_limits_log_.count());
  }
  for (std::size_t i = 0; i < kNumArmJoints; ++i) {
    data_->qpos[arm.qpos.at(i)] = q(static_cast<Eigen::Index>(i));
  }
  kinematics_dirty_ = true;
  return true;
}

void ArmKinematics::ensureForward() const {
  if (!kinematics_dirty_) {
    return;
  }
  // mj_comPos is required on top of mj_kinematics for correct Jacobians (mink does the same).
  mj_kinematics(model_, data_);
  mj_comPos(model_, data_);
  kinematics_dirty_ = false;
}

FramePose ArmKinematics::tcpPose(ArmSide side) const {
  ensureForward();
  const int body = indices(side).tcp_body;
  const mjtNum* pos = data_->xpos + kRotationDim * body;
  const mjtNum* quat = data_->xquat + kQuaternionDim * body;
  FramePose pose;
  pose.position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
  pose.orientation = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
  return pose;
}

ArmJacobian ArmKinematics::tcpJacobianLocal(ArmSide side) const {
  ensureForward();
  const ArmIndices& arm = indices(side);
  const auto nv = static_cast<std::size_t>(model_->nv);
  // MuJoCo returns 3×nv row-major blocks, centred at the body origin but with
  // world-aligned axes.
  std::vector<mjtNum> jacp(kRotationDim * nv);
  std::vector<mjtNum> jacr(kRotationDim * nv);
  mj_jacBody(model_, data_, jacp.data(), jacr.data(), arm.tcp_body);

  Eigen::Matrix<double, kRotationDim, kNumArmJoints> linear_world;
  Eigen::Matrix<double, kRotationDim, kNumArmJoints> angular_world;
  for (int row = 0; row < kRotationDim; ++row) {
    for (std::size_t col = 0; col < kNumArmJoints; ++col) {
      const std::size_t offset = static_cast<std::size_t>(row) * nv +
                                 static_cast<std::size_t>(arm.dof.at(col));
      linear_world(row, static_cast<Eigen::Index>(col)) = jacp.at(offset);
      angular_world(row, static_cast<Eigen::Index>(col)) = jacr.at(offset);
    }
  }
  // Rotate into the body frame (mink: A_fw = Ad(R_wf⁻¹) = blockdiag(Rᵀ, Rᵀ)).
  const Eigen::Map<const Eigen::Matrix<double, kRotationDim, kRotationDim, Eigen::RowMajor>>
      rotation_world_body(data_->xmat + kRotationDim * kRotationDim * arm.tcp_body);
  ArmJacobian jacobian;
  jacobian.topRows<kRotationDim>() = rotation_world_body.transpose() * linear_world;
  jacobian.bottomRows<kRotationDim>() = rotation_world_body.transpose() * angular_world;
  return jacobian;
}

}  // namespace openarm_ik
