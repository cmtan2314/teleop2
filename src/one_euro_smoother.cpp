#include "quest_vr/one_euro_smoother.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

#include "quest_vr/logging.hpp"

namespace quest_vr {
namespace {

Eigen::Vector4d toWxyz(const Eigen::Quaterniond& q) { return {q.w(), q.x(), q.y(), q.z()}; }

Eigen::Quaterniond fromWxyz(const Eigen::Vector4d& v) {
  return Eigen::Quaterniond(v[0], v[1], v[2], v[3]);
}

// Exponential-smoothing factor for a first-order low-pass at cutoff_hz.
double smoothingAlpha(double dt_s, double cutoff_hz) {
  const double tau = 1.0 / (2.0 * std::numbers::pi * cutoff_hz);
  return 1.0 / (1.0 + tau / dt_s);
}

}  // namespace

Eigen::Vector4d slerpWxyz(const Eigen::Vector4d& from, Eigen::Vector4d to, double alpha) {
  double dot = from.dot(to);
  if (dot < 0.0) {
    // q and −q are the same rotation: take the short way round.
    to = -to;
    dot = -dot;
  }
  if (dot > kSlerpLinearThreshold) {
    // Nearly parallel: sin(theta) ≈ 0, use normalized lerp to avoid dividing by ~0.
    const Eigen::Vector4d blended = from + alpha * (to - from);
    return blended / blended.norm();
  }
  const double theta_0 = std::acos(dot);
  const double sin_theta_0 = std::sin(theta_0);
  const double theta = theta_0 * alpha;
  const double sin_theta = std::sin(theta);
  const double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
  const double s1 = sin_theta / sin_theta_0;
  return s0 * from + s1 * to;
}

OneEuroPoseSmoother::OneEuroPoseSmoother(const OneEuroParams& params) : params_(params) {
  if (!(params_.min_cutoff_hz > 0.0) || !(params_.derivative_cutoff_hz > 0.0) ||
      !(params_.beta >= 0.0) || !std::isfinite(params_.min_cutoff_hz) ||
      !std::isfinite(params_.derivative_cutoff_hz) || !std::isfinite(params_.beta)) {
    logf(LogLevel::kError,
         "invalid One Euro params: min_cutoff={} Hz, beta={}, d_cutoff={} Hz (cutoffs must be "
         "> 0, beta >= 0)",
         params_.min_cutoff_hz, params_.beta, params_.derivative_cutoff_hz);
    throw std::invalid_argument("invalid OneEuroParams");
  }
}

void OneEuroPoseSmoother::reset() {
  t_prev_.reset();
  p_prev_.setZero();
  q_prev_ = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  dp_prev_.setZero();
}

Pose OneEuroPoseSmoother::smooth(double t_s, const Pose& target) {
  if (!std::isfinite(t_s)) {
    logf(LogLevel::kWarn, "smooth(): non-finite timestamp, passing pose through unfiltered");
    return target;
  }
  const Eigen::Vector3d& target_p = target.position;
  const Eigen::Vector4d target_q = toWxyz(target.orientation);

  if (!t_prev_.has_value()) {
    // First sample after construction/reset: seed the state, no filtering.
    p_prev_ = target_p;
    q_prev_ = target_q;
    t_prev_ = t_s;
    return target;
  }

  const double dt = t_s - *t_prev_;
  if (dt <= 0.0) {
    // Same or earlier timestamp: python returns the raw target and keeps state.
    logf(LogLevel::kDebug, "smooth(): non-increasing time (dt={} s), passing pose through", dt);
    return target;
  }

  const Eigen::Vector3d dp_raw = (target_p - p_prev_) / dt;
  const double alpha_d = smoothingAlpha(dt, params_.derivative_cutoff_hz);
  const Eigen::Vector3d dp_filtered = alpha_d * dp_raw + (1.0 - alpha_d) * dp_prev_;

  const double speed = dp_filtered.norm();
  const double cutoff_p = params_.min_cutoff_hz + params_.beta * speed;
  const double alpha_p = smoothingAlpha(dt, cutoff_p);

  const Eigen::Vector3d p_hat = p_prev_ + alpha_p * (target_p - p_prev_);
  const Eigen::Vector4d q_hat = slerpWxyz(q_prev_, target_q, alpha_p);

  p_prev_ = p_hat;
  q_prev_ = q_hat;
  dp_prev_ = dp_filtered;
  t_prev_ = t_s;

  Pose out;
  out.position = p_hat;
  out.orientation = fromWxyz(q_hat);
  return out;
}

}  // namespace quest_vr
