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
// so), so the optimum is unique and any exact solver — DAQP or this one — must
// return the same x.
//
// Algorithm: primal active set. Keep a working set of variables pinned to a
// bound, take the Newton step on the free ones, shorten the step at the first
// bound it would cross (and pin that variable), and when the step vanishes,
// release the pinned variable whose Lagrange multiplier has the wrong sign.
// Terminates in finitely many steps for strictly convex problems.
#pragma once

#include "openarm_ik/types.hpp"

namespace openarm_ik {

enum class BoxQpStatus {
  kOptimal,
  kInvalidInput,         // non-finite H/c/bounds, or lower > upper
  kNotPositiveDefinite,  // Cholesky of a free sub-block failed
  kIterationLimit,       // kBoxQpMaxIterations reached (degenerate / cycling)
};
const char* boxQpStatusName(BoxQpStatus status);

struct BoxQpResult {
  BoxQpStatus status = BoxQpStatus::kInvalidInput;
  JointVector solution = JointVector::Zero();  // valid only when status == kOptimal
  int iterations = 0;
  int active_bounds = 0;  // number of variables at a bound at the optimum
};

// Infinite bounds are allowed (unlimited joints). Does not log: it runs inside
// the IK hot loop, so the caller decides how (and how often) to report.
BoxQpResult solveBoxQp(const JointMatrix& hessian, const JointVector& gradient,
                       const JointVector& lower, const JointVector& upper);

}  // namespace openarm_ik
