// Step 2 of the IK port: the SE(3) Lie-group maths behind mink's FrameTask.
//
// Line-by-line port of mink 1.1's native ops (mink/lie/_lie_ops_c.c), which is
// what openarm_control 0.1.0 actually runs. Conventions (same as mink):
//   * a pose T = (R, p) maps frame → world,
//   * twists are [v; ω] (linear first),
//   * log() returns the twist ξ with exp(ξ) = T ("how far, and around which
//     axis, do I have to move to get from identity to T").
//
// Why log() and not "target − current": positions subtract fine, but rotations
// do not — subtracting two quaternions or matrices does not give a rotation.
// log(T_current⁻¹ · T_target) is the one twist that carries the current frame
// onto the target, expressed in the current (TCP) frame, which is exactly the
// frame the body Jacobian of step 1 uses.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "openarm_ik/types.hpp"

namespace openarm_ik {

// [v]× : the matrix with [v]× · u = v × u.
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

// SO(3) log: unit quaternion → rotation vector ω = angle · axis, |angle| ≤ π.
Eigen::Vector3d so3Log(const Eigen::Quaterniond& rotation);
// Inverse of the SO(3) left Jacobian at ω (mink so3_ljacinv).
Eigen::Matrix3d so3LeftJacobianInverse(const Eigen::Vector3d& omega);

// a⁻¹ · b.
FramePose se3InverseMultiply(const FramePose& a, const FramePose& b);
// SE(3) log: pose → twist [v; ω].
Twist se3Log(const FramePose& pose);
// Right-minus a ⊖ b = log(b⁻¹ · a): the twist from b to a, in b's frame.
Twist se3RightMinus(const FramePose& a, const FramePose& b);
// Jacobian of log at T, i.e. the inverse right Jacobian at log(T) (mink se3_jlog).
TwistMatrix se3Jlog(const FramePose& pose);

// mink FrameTask.compute_error: e = target ⊖ current = log(current⁻¹ · target),
// a body twist in the current frame. Zero when the TCP is on target.
Twist frameTaskError(const FramePose& target, const FramePose& current);

// mink FrameTask.compute_jacobian: J_task = −Jlog(target⁻¹ · current) · J_body.
// For small errors Jlog ≈ I, so J_task ≈ −J_body; Jlog corrects the linear
// approximation when the error is large.
ArmJacobian frameTaskJacobian(const FramePose& target, const FramePose& current,
                              const ArmJacobian& body_jacobian);

}  // namespace openarm_ik
