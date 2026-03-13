#pragma once

#include <cmath>

namespace substral {
namespace backend {
namespace solver {

// =============================================================================
// Robust Loss Functions for Bundle Adjustment
// =============================================================================
//
// In standard least squares, we minimize:
//   F(x) = 0.5 * sum_i ||r_i(x)||^2
//
// With a robust loss function rho, we minimize:
//   F(x) = sum_i rho(||r_i(x)||^2)
//
// where rho(s) is applied to the squared residual norm s = ||r_i||^2.
//
// The key insight for integration with the normal equations is the
// Iteratively Reweighted Least Squares (IRLS) approach:
//
//   The gradient of rho(||r||^2) w.r.t. parameters is:
//     d/dx rho(s) = rho'(s) * d/dx s = rho'(s) * 2 * J^T r
//
//   The Gauss-Newton approximation of the Hessian becomes:
//     H ~= 2 * J^T * diag(rho'(s_i)) * J
//
//   This is equivalent to scaling each observation's residual and Jacobian:
//     r_i' = sqrt(rho'(s_i)) * r_i
//     J_i' = sqrt(rho'(s_i)) * J_i
//
//   Then the standard normal equations J'^T J' delta = -J'^T r' give the
//   correct robust update.
//
// Interface:
//   Each loss function provides:
//     rho(s):   the loss value
//     rho'(s):  first derivative (the weight for IRLS)
//
// Note: rho'(s) must be non-negative. For s=0, rho'(0) should be 1
// (matching the un-robustified case for zero residuals).
//
// =============================================================================

// =============================================================================
// Trivial Loss (L2 / standard least squares)
// =============================================================================
//
//   rho(s)  = 0.5 * s
//   rho'(s) = 0.5
//
// The factor of 0.5 matches our convention: cost = sum rho(s_i) = 0.5 * sum s_i
//
// With IRLS weighting: sqrt(rho'(s)) = sqrt(0.5) = 1/sqrt(2)
// But since the 0.5 is a global constant, we can equivalently define:
//
//   rho(s)  = s       (and divide total cost by 2 externally)
//   rho'(s) = 1       (no reweighting)
//
// We use the second convention: rho'(s) = 1, weight = 1, and the cost
// function applies the 0.5 factor externally. This way the trivial loss
// produces exactly the same behavior as the un-robustified solver.
//
// =============================================================================

struct TrivialLoss {
  // rho(s) = s
  static inline double rho(double s) { return s; }

  // rho'(s) = 1
  static inline double weight(double /*s*/) { return 1.0; }
};

// =============================================================================
// Huber Loss
// =============================================================================
//
// Parameterized by delta (the threshold between quadratic and linear regions).
// Applied to s = ||r||^2, so the threshold on ||r|| = delta means s_thresh =
// delta^2.
//
//   rho(s) = s                                    if sqrt(s) <= delta
//          = 2 * delta * sqrt(s) - delta^2        if sqrt(s) > delta
//
//   rho'(s) = 1                                   if sqrt(s) <= delta
//           = delta / sqrt(s)                     if sqrt(s) > delta
//
// Properties:
//   - Continuous and C1 at the transition point
//   - Quadratic near zero (like L2), linear for large residuals
//   - rho'(s) -> 0 as s -> infinity (outliers are down-weighted)
//   - rho'(0) = 1 (inliers get full weight)
//
// =============================================================================

struct HuberLoss {
  double delta;

  explicit HuberLoss(double delta) : delta(delta) {}

  inline double rho(double s) const {
    double r = std::sqrt(s);
    if (r <= delta) {
      return s;
    } else {
      return 2.0 * delta * r - delta * delta;
    }
  }

  inline double weight(double s) const {
    if (s < 1e-30) return 1.0;  // avoid division by zero
    double r = std::sqrt(s);
    if (r <= delta) {
      return 1.0;
    } else {
      return delta / r;
    }
  }
};

// =============================================================================
// Cauchy Loss (Lorentzian)
// =============================================================================
//
// Parameterized by c (the scale parameter).
//
//   rho(s)  = c^2 * log(1 + s / c^2)
//   rho'(s) = 1 / (1 + s / c^2) = c^2 / (c^2 + s)
//
// Properties:
//   - Smooth everywhere
//   - Quadratic near zero: rho(s) ~ s for s << c^2
//   - Logarithmic growth for large s (stronger down-weighting than Huber)
//   - rho'(s) -> 0 as s -> infinity
//   - rho'(0) = 1
//
// =============================================================================

struct CauchyLoss {
  double c;
  double c_sq;

  explicit CauchyLoss(double c) : c(c), c_sq(c * c) {}

  inline double rho(double s) const { return c_sq * std::log(1.0 + s / c_sq); }

  inline double weight(double s) const { return c_sq / (c_sq + s); }
};

}  // namespace solver
}  // namespace backend
}  // namespace substral
