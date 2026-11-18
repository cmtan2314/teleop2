#include "quest_vr/pose_processor.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp>
#include <numbers>
#include <system_error>

#include "quest_vr/constants.hpp"
#include "quest_vr/logging.hpp"

namespace quest_vr {
namespace {

constexpr double kMinQuaternionNorm = 1e-12;

Eigen::Matrix3d vrToRobotMatrix() {
  constexpr int kDim = 3;
  Eigen::Matrix3d rotation;
  for (int row = 0; row < kDim; ++row) {
    for (int col = 0; col < kDim; ++col) {
      rotation(row, col) =
          kVrToRobotRotationRowMajor.at(static_cast<std::size_t>(row * kDim + col));
    }
  }
  return rotation;
}

}  // namespace

std::optional<std::filesystem::path> defaultReferenceCachePath() {
  const char* home = std::getenv("HOME");
  if (home == nullptr || *home == '\0') {
    logf(LogLevel::kWarn, "HOME is not set, reference pose will not be cached to disk");
    return std::nullopt;
  }
  return std::filesystem::path(home) / std::filesystem::path(kReferenceCacheRelPath);
}

std::optional<Pose> unityToRightHanded(const UnityPose& unity_pose) {
  Eigen::Quaterniond orientation(unity_pose.qw, -unity_pose.qx, -unity_pose.qy, unity_pose.qz);
  const double norm = orientation.norm();
  if (!std::isfinite(norm) || norm < kMinQuaternionNorm) {
    logf(LogLevel::kError, "rejecting pose with degenerate quaternion (norm={})", norm);
    return std::nullopt;
  }
  orientation.coeffs() /= norm;  // scipy Rotation.from_quat normalizes too
  Pose pose;
  pose.position = Eigen::Vector3d(unity_pose.x, unity_pose.y, -unity_pose.z);
  pose.orientation = orientation;
  return pose;
}

QuestPoseProcessor::QuestPoseProcessor(std::optional<std::filesystem::path> reference_cache_path)
    : reference_cache_path_(std::move(reference_cache_path)),
      vr_to_robot_(vrToRobotMatrix()),
      frame_offset_(kFrameOffsetNeckM[0], kFrameOffsetNeckM[1], kFrameOffsetNeckM[2]),
      controller_fix_(Eigen::AngleAxisd(kControllerFixYawDeg * std::numbers::pi / 180.0,
                                        Eigen::Vector3d::UnitZ())) {
  vr_to_robot_.normalize();
  const double det = vrToRobotMatrix().determinant();
  constexpr double kRotationDetTolerance = 1e-9;
  if (std::abs(det - 1.0) > kRotationDetTolerance) {
    // A reflection here would mirror every rotation (see the LH→RH discussion).
    logf(LogLevel::kError, "kVrToRobotRotationRowMajor is not a proper rotation (det={})", det);
  }
}

QuestPoseProcessor::Result QuestPoseProcessor::process(const QuestPacket& packet) {
  if (!reference_.has_value()) {
    captureReference(packet);
  }
  Result result;
  if (packet.right_controller.has_value()) {
    result.right = rectify(*packet.right_controller, "right");
  }
  if (packet.left_controller.has_value()) {
    result.left = rectify(*packet.left_controller, "left");
  }
  return result;
}

void QuestPoseProcessor::captureReference(const QuestPacket& packet) {
  UnityPose unity_reference;  // identity (python: _IDENTITY_REF)
  if (packet.reference.has_value()) {
    unity_reference = *packet.reference;
    saveReference(unity_reference);
  } else {
    logf(LogLevel::kWarn, "first packet has no 'rf' reference, using identity reference");
  }

  std::optional<Pose> reference = unityToRightHanded(unity_reference);
  if (!reference.has_value()) {
    logf(LogLevel::kError, "reference pose is degenerate, falling back to identity reference");
    reference = unityToRightHanded(UnityPose{});
  }
  if (!reference.has_value()) {
    logf(LogLevel::kError, "identity reference unexpectedly rejected, reference not captured");
    return;
  }
  reference_ = Reference{reference->position, reference->orientation.conjugate()};
  logf(LogLevel::kInfo, "reference captured: p=[{:.4f}, {:.4f}, {:.4f}]",
       reference->position.x(), reference->position.y(), reference->position.z());
}

void QuestPoseProcessor::saveReference(const UnityPose& unity_reference) const {
  if (!reference_cache_path_.has_value()) {
    logf(LogLevel::kDebug, "reference cache disabled, not writing reference pose");
    return;
  }
  const std::filesystem::path& path = *reference_cache_path_;
  std::error_code error;
  std::filesystem::create_directories(path.parent_path(), error);
  if (error) {
    logf(LogLevel::kWarn, "cannot create '{}': {}", path.parent_path().string(), error.message());
    return;
  }
  std::ofstream file(path);
  if (!file) {
    logf(LogLevel::kWarn, "cannot open '{}' for writing reference pose", path.string());
    return;
  }
  const nlohmann::json json = {
      {"x", unity_reference.x},   {"y", unity_reference.y},   {"z", unity_reference.z},
      {"qx", unity_reference.qx}, {"qy", unity_reference.qy}, {"qz", unity_reference.qz},
      {"qw", unity_reference.qw},
  };
  file << json.dump();
  logf(LogLevel::kInfo, "reference pose written to '{}'", path.string());
}

std::optional<Pose> QuestPoseProcessor::rectify(const UnityPose& unity_pose,
                                                const char* side) const {
  if (!reference_.has_value()) {
    logf(LogLevel::kError, "rectify({}) called before a reference was captured", side);
    return std::nullopt;
  }
  const std::optional<Pose> controller = unityToRightHanded(unity_pose);
  if (!controller.has_value()) {
    logf(LogLevel::kWarn, "{} controller pose dropped (degenerate quaternion)", side);
    return std::nullopt;
  }
  const Eigen::Vector3d relative_position =
      reference_->inverse_orientation * (controller->position - reference_->position);
  const Eigen::Quaterniond relative_orientation =
      reference_->inverse_orientation * controller->orientation;

  Pose out;
  out.position = vr_to_robot_ * relative_position + frame_offset_;
  out.orientation = (vr_to_robot_ * relative_orientation * controller_fix_).normalized();
  return out;
}

}  // namespace quest_vr
