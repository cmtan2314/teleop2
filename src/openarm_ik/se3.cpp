#include "openarm_ik/se3.hpp"

#include <cmath>

#include "openarm_ik/constants.hpp"

namespace openarm_ik {
namespace {

constexpr int kRotationDim = 3;

}  // namespace

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),  //
      v.z(), 0.0, -v.x(),   //
      -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Vector3d so3Log(const Eigen::Quaterniond& rotation) {
  // q and −q are the same rotation; pick w ≥ 0 so the angle lands in [0, π].
  Eigen::Quaterniond q = rotation;
  if (q.w() < 0.0) {
    q.coeffs() = -q.coeffs();
  }
  const Eigen::Vector3d vec = q.vec();
  const double sin_half_angle = vec.norm();
  if (sin_half_angle < kLieSmallAngleEpsilon) {
    return Eigen::Vector3d::Zero();  // identity rotation: axis undefined, ω = 0
  }
  const double angle = 2.0 * std::atan2(sin_half_angle, q.w());
  return angle * (vec / sin_half_angle);
}

Eigen::Matrix3d so3LeftJacobianInverse(const Eigen::Vector3d& omega) {
  const double theta = omega.norm();
  const double theta_sq = theta * theta;
  double beta = 0.0;
  if (theta < kLieSmallAngleEpsilon) {
    // Taylor series of the expression below around θ = 0 (mink so3_ljacinv).
    beta = (1.0 / 12.0) *
           (1.0 + theta_sq / 60.0 * (1.0 + theta_sq / 42.0 * (1.0 + theta_sq / 40.0)));
  } else {
    beta = (1.0 / theta_sq) *
           (1.0 - (theta * std::sin(theta) / (2.0 * (1.0 - std::cos(theta)))));
  }
  const Eigen::Matrix3d outer =
      omega * omega.transpose() - omega.squaredNorm() * Eigen::Matrix3d::Identity();
  return Eigen::Matrix3d::Identity() + beta * outer - 0.5 * skew(omega);
}

FramePose se3InverseMultiply(const FramePose& a, const FramePose& b) {
  const Eigen::Quaterniond a_inverse = a.orientation.conjugate();
  FramePose out;
  out.orientation = a_inverse * b.orientation;
  out.position = a_inverse * (b.position - a.position);
  return out;
}

Twist se3Log(const FramePose& pose) {
  const Eigen::Vector3d omega = so3Log(pose.orientation);
  const double theta = omega.norm();
  const double theta_sq = theta * theta;
  const Eigen::Matrix3d skew_omega = skew(omega);
  const Eigen::Matrix3d skew_omega_sq = skew_omega * skew_omega;

  // V⁻¹: maps the translation onto the linear part of the twist.
  double coeff = 0.0;
  if (theta_sq < kLieSmallAngleEpsilon) {
    coeff = 1.0 / 12.0;  // limit of the expression below as θ → 0
  } else {
    const double half_theta = 0.5 * theta;
    coeff = (1.0 - 0.5 * theta * std::cos(half_theta) / std::sin(half_theta)) / theta_sq;
  }
  const Eigen::Matrix3d v_inverse =
      Eigen::Matrix3d::Identity() - 0.5 * skew_omega + coeff * skew_omega_sq;

  Twist twist;
  twist.head<kRotationDim>() = v_inverse * pose.position;
  twist.tail<kRotationDim>() = omega;
  return twist;
}

Twist se3RightMinus(const FramePose& a, const FramePose& b) {
  return se3Log(se3InverseMultiply(b, a));
}

namespace {

// Q(ξ) block of the SE(3) left Jacobian (Sola et al., eqn 180; mink _getQ).
Eigen::Matrix3d se3JacobianQ(const Twist& twist) {
  const Eigen::Vector3d v = twist.head<kRotationDim>();
  const Eigen::Vector3d w = twist.tail<kRotationDim>();
  const double theta = w.norm();
  const double t2 = theta * theta;
  constexpr double kA = 0.5;
  double b = 0.0;
  double c = 0.0;
  double d = 0.0;
  if (t2 < kLieSmallAngleEpsilon) {
    // Taylor series around θ = 0.
    b = (1.0 / 6.0) + (1.0 / 120.0) * t2;
    c = -(1.0 / 24.0) + (1.0 / 720.0) * t2;
    d = -(1.0 / 60.0);
  } else {
    const double t4 = t2 * t2;
    const double sin_t = std::sin(theta);
    const double cos_t = std::cos(theta);
    b = (theta - sin_t) / (t2 * theta);
    c = (1.0 - 0.5 * t2 - cos_t) / t4;
    d = (2.0 * theta - 3.0 * sin_t + theta * cos_t) / (2.0 * t4 * theta);
  }
  const Eigen::Matrix3d big_v = skew(v);
  const Eigen::Matrix3d big_w = skew(w);
  const Eigen::Matrix3d vw = big_v * big_w;
  const Eigen::Matrix3d wv = vw.transpose();  // (VW)ᵀ = WV for skew matrices
  const Eigen::Matrix3d wvw = wv * big_w;
  const Eigen::Matrix3d vww = vw * big_w;
  return kA * big_v + b * (wv + vw + wvw) - c * (vww - vww.transpose() - 3.0 * wvw) +
         d * (wvw * big_w + big_w * wvw);
}

}  // namespace

TwistMatrix se3Jlog(const FramePose& pose) {
  // Jlog(T) = Jr⁻¹(log T) = Jl⁻¹(−log T).
  const Twist negated = -se3Log(pose);
  const Eigen::Vector3d negated_omega = negated.tail<kRotationDim>();
  if (negated_omega.squaredNorm() < kLieSmallAngleEpsilon) {
    return TwistMatrix::Identity();
  }
  const Eigen::Matrix3d jl_inverse = so3LeftJacobianInverse(negated_omega);
  const Eigen::Matrix3d q = se3JacobianQ(negated);
  TwistMatrix jlog = TwistMatrix::Zero();
  jlog.topLeftCorner<kRotationDim, kRotationDim>() = jl_inverse;
  jlog.bottomRightCorner<kRotationDim, kRotationDim>() = jl_inverse;
  jlog.topRightCorner<kRotationDim, kRotationDim>() = -jl_inverse * q * jl_inverse;
  return jlog;
}

Twist frameTaskError(const FramePose& target, const FramePose& current) {
  return se3RightMinus(target, current);
}

ArmJacobian frameTaskJacobian(const FramePose& target, const FramePose& current,
                              const ArmJacobian& body_jacobian) {
  const FramePose target_to_current = se3InverseMultiply(target, current);
  return -se3Jlog(target_to_current) * body_jacobian;
}

}  // namespace openarm_ik
