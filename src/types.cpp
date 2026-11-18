#include "quest_vr/types.hpp"

namespace quest_vr {

const char* validityName(Validity validity) {
  switch (validity) {
    case Validity::kOk:
      return "OK";
    case Validity::kStale:
      return "STALE";
    case Validity::kInvalid:
      return "INVALID";
  }
  return "UNKNOWN";
}

std::array<double, kFlatPoseSize> toFlatPose(const Pose& pose) {
  const Eigen::Vector3d& p = pose.position;
  const Eigen::Quaterniond& q = pose.orientation;
  return {p.x(), p.y(), p.z(), q.w(), q.x(), q.y(), q.z()};
}

}  // namespace quest_vr
