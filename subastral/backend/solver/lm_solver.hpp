#pragma once

#include <Eigen/Dense>
#include <functional>
#include <iostream>

#include "subastral/backend/common.h"
#include "subastral/backend/solver/loss_function.hpp"
#include "subastral/backend/solver/schur.hpp"

namespace substral {
namespace backend {
namespace solver {

// =============================================================================
// Levenberg-Marquardt Solver for Bundle Adjustment
// =============================================================================
//
// The LM algorithm is a trust-region method for nonlinear least squares.
// At each iteration it solves:
//
//   (J^T J + λ · diag(J^T J)) · δ = -J^T r
//
// where λ is the damping parameter. When λ is large, the step is small
// and close to gradient descent. When λ is small, the step is close to
// Gauss-Newton.
//
// The damping strategy follows Nielsen (1999):
//   - If the step reduces the cost: accept, decrease λ by factor
//   - If the step increases the cost: reject, increase λ by factor
//
// We use the Schur complement to efficiently solve the normal equations
// by eliminating the point variables.
//
// =============================================================================

// =============================================================================
// Loss function type enum (shared between CPU and GPU solvers)
// =============================================================================
enum class LossType {
  TRIVIAL = 0,  // Standard L2 (no robustification)
  HUBER = 1,    // Huber loss: quadratic near zero, linear for large residuals
  CAUCHY = 2,   // Cauchy/Lorentzian: logarithmic growth, heavy down-weighting
};

// =============================================================================
// Linear solver type for the inner linear system solve
// =============================================================================
enum class LinearSolverType {
  CHOLESKY = 0,  // Sparse Cholesky (cusolverSpDcsrlsvchol) — requires SPD
  PCG = 1,  // Preconditioned Conjugate Gradient — handles indefinite systems
};

struct LMConfig {
  double initial_lambda = 1e-3;
  double lambda_factor = 10.0;  // multiply/divide λ by this
  double min_lambda = 1e-10;
  double max_lambda = 1e10;

  int max_iterations = 50;
  double gradient_tolerance = 1e-10;  // stop if ||g||_inf < tol
  double step_tolerance = 1e-10;      // stop if ||δ|| / ||x|| < tol
  double cost_tolerance = 1e-10;      // stop if |Δcost| / cost < tol

  // Robust loss function configuration
  LossType loss_type = LossType::TRIVIAL;
  double loss_param = 1.0;  // delta for Huber, c for Cauchy

  // SE(3) Lie group parameterization for camera poses.
  // When true:
  //   - Jacobians use left-perturbation model (se(3) tangent space)
  //   - Camera pose updates use exponential map: T_new = Exp(δξ) · T_old
  //   - Intrinsics and points still use additive updates
  // When false (default):
  //   - Classical global angle-axis parameterization with additive updates
  bool use_lie = false;

  // Linear solver for the inner system (H + λI) δ = -g.
  // CHOLESKY: sparse Cholesky via cuSOLVER (fast, requires SPD).
  // PCG: Preconditioned Conjugate Gradient (handles indefinite systems).
  LinearSolverType linear_solver = LinearSolverType::CHOLESKY;

  // PCG-specific parameters (ignored when linear_solver == CHOLESKY)
  int pcg_max_iterations = 500;  // max CG iterations per linear solve
  double pcg_tolerance = 1e-8;   // relative residual tolerance for CG

  bool verbose = true;
};

struct LMResult {
  double initial_cost;
  double final_cost;
  int iterations;
  bool converged;
  std::string termination_reason;
};

// The solver operates on a BAProblem in-place, modifying camera and point
// parameters to minimize the reprojection error.
//
// Internally it:
//   1. Computes residuals and Jacobians (CPU, using projectWithJacobianCPU)
//   2. Accumulates the block-sparse normal equations
//   3. Forms the Schur complement (camera-only system)
//   4. Adds LM damping to the diagonal
//   5. Solves the reduced system with Eigen's dense solver
//   6. Back-substitutes for point updates
//   7. Evaluates the new cost and accepts/rejects the step
//
LMResult solveLM(BAProblem& problem, const LMConfig& config = LMConfig());

}  // namespace solver
}  // namespace backend
}  // namespace substral
