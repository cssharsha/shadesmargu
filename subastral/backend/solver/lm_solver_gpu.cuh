#pragma once

#include <cuda_runtime.h>

#include <iostream>
#include <string>

#include "subastral/backend/common.h"
#include "subastral/backend/solver/lm_solver.hpp"

namespace substral {
namespace backend {
namespace solver {

// =============================================================================
// GPU Levenberg-Marquardt Solver for Bundle Adjustment
// =============================================================================
//
// Full GPU pipeline:
//   1. Upload problem data to device (once)
//   2. LM loop (entirely on GPU):
//      a. GPU: compute residuals + Jacobians (existing kernel, 1 thread/obs)
//      b. GPU: accumulate normal equations (1 thread/obs, atomicAdd)
//      c. GPU: initialize S = U, rhs = g_c (1 thread/cam)
//      d. GPU: Schur complement subtraction (1 thread/pt, atomicAdd into S)
//      e. GPU: apply LM damping to S diagonal copy
//      f. GPU: cuSOLVER Cholesky factorize + solve for delta_cameras
//      g. GPU: back-substitute for delta_points (1 thread/pt)
//      h. GPU: save current params (cudaMemcpy D2D)
//      i. GPU: apply parameter update (1 thread per element)
//      j. GPU: compute new cost (projection + squared residuals + reduce)
//      k. Accept/reject: if reject, restore saved params (cudaMemcpy D2D)
//   3. Download final parameters to host (once)
//
// The dense Schur complement solve uses cuSOLVER's Cholesky factorization
// (dpotrf + dpotrs) on GPU, eliminating D2H/H2D transfers. The LM-damped
// Schur complement is SPD by construction, making Cholesky valid. If
// factorization fails (rare), falls back to Eigen LDLT on CPU.
//
// =============================================================================

// Reuses LMConfig and LMResult from lm_solver.hpp

LMResult solveLM_GPU(BAProblem& problem, const LMConfig& config = LMConfig());

}  // namespace solver
}  // namespace backend
}  // namespace substral
