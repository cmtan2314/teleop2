#include "openarm_ik/box_qp.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "openarm_ik/constants.hpp"

namespace openarm_ik {
namespace {

enum class BoundState { kFree, kAtLower, kAtUpper };

// Fixed-capacity dynamic sizes: the free sub-problem has 0..7 variables and
// must not allocate on the heap (this runs 20× per IK tick).
using FreeVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kNumArmJoints, 1>;
using FreeMatrix =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, kNumArmJoints, kNumArmJoints>;

constexpr auto kNumVars = static_cast<Eigen::Index>(kNumArmJoints);

}  // namespace

const char* boxQpStatusName(BoxQpStatus status) {
  switch (status) {
    case BoxQpStatus::kOptimal:
      return "optimal";
    case BoxQpStatus::kInvalidInput:
      return "invalid input";
    case BoxQpStatus::kNotPositiveDefinite:
      return "Hessian not positive definite";
    case BoxQpStatus::kIterationLimit:
      return "iteration limit";
  }
  return "unknown";
}

BoxQpResult solveBoxQp(const JointMatrix& hessian, const JointVector& gradient,
                       const JointVector& lower, const JointVector& upper) {
  BoxQpResult result;
  // NaN bounds are rejected; ±inf bounds (unlimited joints) are fine.
  if (!hessian.allFinite() || !gradient.allFinite() || lower.hasNaN() || upper.hasNaN() ||
      (lower.array() > upper.array()).any()) {
    result.status = BoxQpStatus::kInvalidInput;
    return result;
  }

  // Feasible start: x = clamp(0). Variables that had to be clamped start pinned.
  JointVector x = JointVector::Zero();
  std::array<BoundState, kNumArmJoints> state{};
  for (Eigen::Index i = 0; i < kNumVars; ++i) {
    const auto k = static_cast<std::size_t>(i);
    if (lower(i) >= 0.0) {
      x(i) = lower(i);
      state[k] = BoundState::kAtLower;
    } else if (upper(i) <= 0.0) {
      x(i) = upper(i);
      state[k] = BoundState::kAtUpper;
    } else {
      state[k] = BoundState::kFree;
    }
  }

  for (int iteration = 1; iteration <= kBoxQpMaxIterations; ++iteration) {
    result.iterations = iteration;
    const JointVector grad = hessian * x + gradient;  // ∇ of the objective at x

    std::array<Eigen::Index, kNumArmJoints> free_index{};
    Eigen::Index num_free = 0;
    for (Eigen::Index i = 0; i < kNumVars; ++i) {
      if (state[static_cast<std::size_t>(i)] == BoundState::kFree) {
        free_index[static_cast<std::size_t>(num_free++)] = i;
      }
    }

    // Newton step on the free variables: H_FF · p = −grad_F.
    FreeVector step(num_free);
    if (num_free > 0) {
      FreeMatrix h_free(num_free, num_free);
      FreeVector g_free(num_free);
      for (Eigen::Index r = 0; r < num_free; ++r) {
        const Eigen::Index row = free_index[static_cast<std::size_t>(r)];
        g_free(r) = grad(row);
        for (Eigen::Index c = 0; c < num_free; ++c) {
          h_free(r, c) = hessian(row, free_index[static_cast<std::size_t>(c)]);
        }
      }
      const Eigen::LLT<FreeMatrix> llt(h_free);
      if (llt.info() != Eigen::Success) {
        result.status = BoxQpStatus::kNotPositiveDefinite;
        return result;
      }
      step = llt.solve(-g_free);
    }

    const double step_size = num_free > 0 ? step.cwiseAbs().maxCoeff() : 0.0;
    if (step_size <= kBoxQpStepTolerance) {
      // x is optimal on this face. Check the multipliers of the pinned bounds:
      // at a lower bound the objective must not decrease when moving up
      // (grad ≥ 0), at an upper bound when moving down (grad ≤ 0).
      Eigen::Index release = -1;
      double most_negative = -kBoxQpDualTolerance;
      for (Eigen::Index i = 0; i < kNumVars; ++i) {
        const BoundState s = state[static_cast<std::size_t>(i)];
        if (s == BoundState::kFree || lower(i) == upper(i)) {
          continue;  // equal bounds: the variable is fixed, never released
        }
        const double multiplier = (s == BoundState::kAtLower) ? grad(i) : -grad(i);
        if (multiplier < most_negative) {
          most_negative = multiplier;
          release = i;
        }
      }
      if (release < 0) {
        result.status = BoxQpStatus::kOptimal;
        result.solution = x;
        result.active_bounds = static_cast<int>(kNumVars - num_free);
        return result;
      }
      state[static_cast<std::size_t>(release)] = BoundState::kFree;
      continue;
    }

    // Walk along the step until the first bound is hit (or the full step).
    double alpha = 1.0;
    Eigen::Index blocking = -1;
    BoundState blocking_state = BoundState::kFree;
    for (Eigen::Index r = 0; r < num_free; ++r) {
      const Eigen::Index i = free_index[static_cast<std::size_t>(r)];
      if (step(r) < 0.0 && std::isfinite(lower(i))) {
        const double to_bound = (lower(i) - x(i)) / step(r);
        if (to_bound < alpha) {
          alpha = to_bound;
          blocking = i;
          blocking_state = BoundState::kAtLower;
        }
      } else if (step(r) > 0.0 && std::isfinite(upper(i))) {
        const double to_bound = (upper(i) - x(i)) / step(r);
        if (to_bound < alpha) {
          alpha = to_bound;
          blocking = i;
          blocking_state = BoundState::kAtUpper;
        }
      }
    }
    alpha = std::max(alpha, 0.0);
    for (Eigen::Index r = 0; r < num_free; ++r) {
      x(free_index[static_cast<std::size_t>(r)]) += alpha * step(r);
    }
    if (blocking >= 0) {
      // Snap exactly onto the bound so rounding cannot leave it slightly outside.
      x(blocking) = (blocking_state == BoundState::kAtLower) ? lower(blocking) : upper(blocking);
      state[static_cast<std::size_t>(blocking)] = blocking_state;
    }
  }
  result.status = BoxQpStatus::kIterationLimit;
  return result;
}

}  // namespace openarm_ik
