// Port of QuestPoseProcessor (quest_receiver.py): Unity left-handed poses →
// right-handed, expressed relative to a reference (headset) pose captured once,
// then mapped into the robot workspace:
//
//   p_rel = R_ref⁻¹ (p_ctrl − p_ref)         r_rel = R_ref⁻¹ · r_ctrl
//   p_out = R_FRAME · p_rel + FRAME_OFFSET   r_out = R_FRAME · r_rel · R_FIX
#pragma once

#include <filesystem>
#include <optional>

#include "quest_vr/packet.hpp"
#include "quest_vr/types.hpp"

namespace quest_vr {

// $HOME/.cache/dora_openarm_vr/reference_pose.json, or nullopt if HOME is unset.
std::optional<std::filesystem::path> defaultReferenceCachePath();

// Unity left-handed → right-handed by mirroring z:
//   p = [x, y, −z],  q(x, y, z, w) = [−qx, −qy, qz, qw]
// Returns nullopt (and logs) if the quaternion has zero norm.
std::optional<Pose> unityToRightHanded(const UnityPose& unity_pose);

class QuestPoseProcessor {
 public:
  struct Result {
    std::optional<Pose> right;  // nullopt if "rc" absent or malformed
    std::optional<Pose> left;   // nullopt if "lc" absent or malformed
  };

  // reference_cache_path: where to write the captured reference for inspection
  // (nullopt disables writing).
  explicit QuestPoseProcessor(std::optional<std::filesystem::path> reference_cache_path);

  // The reference is captured from the first packet ever processed and then
  // frozen for the life of this object (no re-capture on re-engagement).
  Result process(const QuestPacket& packet);

  bool hasReference() const { return reference_.has_value(); }

 private:
  void captureReference(const QuestPacket& packet);
  void saveReference(const UnityPose& unity_reference) const;
  std::optional<Pose> rectify(const UnityPose& unity_pose, const char* side) const;

  std::optional<std::filesystem::path> reference_cache_path_;
  struct Reference {
    Eigen::Vector3d position;
    Eigen::Quaterniond inverse_orientation;
  };
  std::optional<Reference> reference_;

  Eigen::Quaterniond vr_to_robot_;     // R_FRAME
  Eigen::Vector3d frame_offset_;       // FRAME_OFFSET_NECK
  Eigen::Quaterniond controller_fix_;  // R_FIX
};

}  // namespace quest_vr
