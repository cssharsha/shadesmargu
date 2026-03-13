#pragma once

#include <cuda_runtime.h>

#include <functional>
#include <string>
#include <vector>

#include "subastral/backend/common.h"
#include "subastral/backend/solver/lm_solver.hpp"

namespace substral {
namespace backend {
namespace solver {

// =============================================================================
// GPU Pose-Graph Solver — Levenberg-Marquardt with Sparse Cholesky
// =============================================================================
//
// Full GPU pipeline:
//   1. CPU (once): build CSR sparsity pattern from edge connectivity
//   2. GPU: compute per-edge residuals (1 thread/edge)
//   3. GPU: compute per-edge Jacobians (1 thread/edge)
//   4. GPU: accumulate Hessian into CSR values (1 thread/edge, atomicAdd)
//   5. GPU: apply LM damping to diagonal blocks
//   6. GPU: cusolverSpDcsrlsvchol — sparse Cholesky solve
//   7. GPU: update poses via exp(δξ) · T (1 thread/pose)
//   8. GPU: evaluate new cost → accept/reject
//
// Reuses LMConfig and LMResult from lm_solver.hpp.
//
// =============================================================================

// Callback: called after each iteration with (iteration, cost).
// Return false to abort.
using PoseGraphCallback = std::function<bool(int, double)>;

LMResult solvePoseGraph_GPU(
    PoseGraphProblem& problem,
    const LMConfig& config = LMConfig(),
    PoseGraphCallback callback = nullptr);

}  // namespace solver
}  // namespace backend
}  // namespace substral
