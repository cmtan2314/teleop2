// Port of smoothing.py: One Euro filter on position (speed-adaptive cutoff) and
// SLERP on orientation using the same alpha.
//
//   alpha(dt, fc) = 1 / (1 + tau/dt),  tau = 1 / (2π fc)
//   fc_pos        = min_cutoff + beta · |filtered velocity|
#pragma once

#include <Eigen/Core>
#include <optional>

#include "quest_vr/constants.hpp"
#include "quest_vr/types.hpp"

namespace quest_vr {

struct OneEuroParams {
  double min_cutoff_hz = kSmootherMinCutoffHz;
  double beta = kSmootherBeta;
  double derivative_cutoff_hz = kSmootherDerivativeCutoffHz;
};

// Quaternions are handled as raw [w, x, y, z] 4-vectors, exactly like the
// python implementation, so results match it numerically.
Eigen::Vector4d slerpWxyz(const Eigen::Vector4d& from, Eigen::Vector4d to, double alpha);

class OneEuroPoseSmoother {
 public:
  // Throws std::invalid_argument (after logging) if a parameter is out of range.
  explicit OneEuroPoseSmoother(const OneEuroParams& params = {});

  // Clear state so the next sample is passed through unfiltered.
  void reset();

  // t_s: monotonic time in seconds. Returns the filtered pose.
  Pose smooth(double t_s, const Pose& target);

 private:
  OneEuroParams params_;
  std::optional<double> t_prev_;
  Eigen::Vector3d p_prev_ = Eigen::Vector3d::Zero();
  Eigen::Vector4d q_prev_ = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d dp_prev_ = Eigen::Vector3d::Zero();
};

}  // namespace quest_vr
