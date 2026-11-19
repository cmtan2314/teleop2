// Step 3 of the IK port: the quadratic program mink hands to DAQP, per arm.
//
//   minimise   ½ xᵀ H x + cᵀ x
//   subject to lower ≤ x ≤ upper          (x = Δq of one arm, 7 joints)
//
// In openarm_control 0.1.0 the QP over all 18 DOFs splits exactly into one
// such problem per arm: the Hessian is block-diagonal per arm, the joint-limit
// constraints are plain bounds, and the finger DOFs are frozen (Δq = 0).
//
// H is strictly positive definite (the Tikhonov `damping` term alone makes it
// so), so the optimum is unique and any exact solver must return the same x.
//
// Two interchangeable backends:
//   * kDaqp (default): DAQP 0.7.2, the very C solver python reaches through
//     qpsolvers (vendored in third_party/daqp). Bounds go in as DAQP "simple
//     bounds"; ±inf becomes ±DAQP_INF (1e30) like qpsolvers does.
//   * kActiveSet: a self-written primal active-set solver. Keep a working set
//     of variables pinned to a bound, take the Newton step on the free ones,
//     shorten it at the first bound it would cross (and pin that variable),
//     and when the step vanishes release the pinned variable whose Lagrange
//     multiplier has the wrong sign. No heap allocation.
#pragma once

#include "openarm_ik/types.hpp"

namespace openarm_ik {

enum class QpBackend { kDaqp, kActiveSet };
const char* qpBackendName(QpBackend backend);

enum class BoxQpStatus {
  kOptimal,
  kInvalidInput,         // non-finite H/c, NaN bounds, or lower > upper
  kNotPositiveDefinite,  // active set: Cholesky of a free sub-block failed
  kIterationLimit,       // active set: kBoxQpMaxIterations reached
  kSolverFailed,         // DAQP returned exitflag ≤ 0 (see solver_exitflag)
};
const char* boxQpStatusName(BoxQpStatus status);

struct BoxQpResult {
  BoxQpStatus status = BoxQpStatus::kInvalidInput;
  JointVector solution = JointVector::Zero();  // valid only when status == kOptimal
  int iterations = 0;
  int active_bounds = 0;    // active set only: variables at a bound at the optimum
  int solver_exitflag = 0;  // DAQP only: raw exitflag (1 optimal, <0 failure)
};

// Infinite bounds are allowed (unlimited joints). Does not log: it runs inside
// the IK hot loop, so the caller decides how (and how often) to report.
BoxQpResult solveBoxQp(const JointMatrix& hessian, const JointVector& gradient,
                       const JointVector& lower, const JointVector& upper,
                       QpBackend backend = QpBackend::kDaqp);

}  // namespace openarm_ik
