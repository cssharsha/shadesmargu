#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace substral {
namespace backend {
namespace lie {

// =============================================================================
// SO(3) Lie Group Operations — CPU Reference Implementation
// =============================================================================
//
// SO(3) is the group of 3D rotations: {R ∈ ℝ³ˣ³ | R^T R = I, det R = +1}
//
// Its Lie algebra so(3) consists of 3×3 skew-symmetric matrices:
//   Ω = [ω]× ∈ so(3)  ↔  ω ∈ ℝ³
//
// The exponential map sends so(3) → SO(3):
//   exp([ω]×) = R(ω)
//
// The logarithm map sends SO(3) → so(3):
//   log(R) = [ω]×  where ω is the angle-axis representation
//
// =============================================================================

// =============================================================================
// hat: ℝ³ → so(3)
// =============================================================================
//
// Maps a 3-vector to its skew-symmetric matrix:
//
//   hat(ω) = [ω]× = |  0    -ω₂   ω₁ |
//                    |  ω₂    0   -ω₀ |
//                    | -ω₁   ω₀    0  |
//
// Property: [ω]× · v = ω × v  (cross product)
//
inline Eigen::Matrix3d hat(const Eigen::Vector3d& w) {
  Eigen::Matrix3d W;
  W << 0.0, -w(2), w(1), w(2), 0.0, -w(0), -w(1), w(0), 0.0;
  return W;
}

// =============================================================================
// vee: so(3) → ℝ³
// =============================================================================
//
// Inverse of hat — extracts the 3-vector from a skew-symmetric matrix:
//
//   vee([ω]×) = ω = (W(2,1), W(0,2), W(1,0))
//
inline Eigen::Vector3d vee(const Eigen::Matrix3d& W) {
  return Eigen::Vector3d(W(2, 1), W(0, 2), W(1, 0));
}

// =============================================================================
// exp: so(3) → SO(3)  (Rodrigues formula)
// =============================================================================
//
// Given ω ∈ ℝ³ with θ = ‖ω‖ and k = ω/θ:
//
//   exp([ω]×) = I + (sin θ / θ) [ω]× + ((1 - cos θ) / θ²) [ω]×²
//
// This is the Rodrigues formula. The coefficients have Taylor series:
//
//   sin θ / θ        → 1 - θ²/6 + θ⁴/120 - ...
//   (1 - cos θ) / θ² → 1/2 - θ²/24 + θ⁴/720 - ...
//
// Small-angle case (θ < ε):
//   exp([ω]×) ≈ I + [ω]×
//
// =============================================================================

inline Eigen::Matrix3d expSO3(const Eigen::Vector3d& w) {
  double theta_sq = w.squaredNorm();
  double theta = std::sqrt(theta_sq);

  Eigen::Matrix3d W = hat(w);

  if (theta < 1e-10) {
    // First-order approximation: exp ≈ I + [ω]×
    return Eigen::Matrix3d::Identity() + W;
  }

  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);

  // Rodrigues: R = I + (sinθ/θ)[ω]× + ((1-cosθ)/θ²)[ω]×²
  double a = sin_theta / theta;  // sinc(θ)
  double b = (1.0 - cos_theta) / theta_sq;

  return Eigen::Matrix3d::Identity() + a * W + b * (W * W);
}

// =============================================================================
// log: SO(3) → so(3)  (inverse of exp)
// =============================================================================
//
// Given R ∈ SO(3), find ω such that exp([ω]×) = R.
//
// The rotation angle θ satisfies:
//   cos θ = (tr(R) - 1) / 2
//
// Case 1: θ ≈ 0 (R ≈ I)
//   [ω]× ≈ (R - R^T) / 2
//   ω = vee((R - R^T) / 2)
//
// Case 2: θ ≈ π (tr(R) ≈ -1)
//   The axis k is the eigenvector of R corresponding to eigenvalue 1.
//   We extract it from the column of (R + I) with largest norm.
//   ω = θ · k
//
// Case 3: General case
//   [ω]× = (θ / (2 sin θ)) · (R - R^T)
//   ω = vee([ω]×)
//
// =============================================================================

inline Eigen::Vector3d logSO3(const Eigen::Matrix3d& R) {
  double trace = R.trace();
  double cos_theta = (trace - 1.0) / 2.0;

  // Clamp for numerical safety
  cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
  double theta = std::acos(cos_theta);

  // Case 1: θ ≈ 0
  if (theta < 1e-10) {
    // First-order: [ω]× ≈ (R - R^T)/2
    return vee((R - R.transpose()) / 2.0);
  }

  // Case 2: θ ≈ π
  if (theta > M_PI - 1e-6) {
    // Find the column of (R + I) with the largest norm
    // This gives us the rotation axis (up to sign)
    Eigen::Matrix3d RpI = R + Eigen::Matrix3d::Identity();
    int best_col = 0;
    double best_norm = RpI.col(0).squaredNorm();
    for (int c = 1; c < 3; ++c) {
      double n = RpI.col(c).squaredNorm();
      if (n > best_norm) {
        best_norm = n;
        best_col = c;
      }
    }
    Eigen::Vector3d k = RpI.col(best_col).normalized();
    return theta * k;
  }

  // Case 3: General case
  double sin_theta = std::sin(theta);
  double factor = theta / (2.0 * sin_theta);
  return vee(factor * (R - R.transpose()));
}

// =============================================================================
// Left Jacobian of SO(3): J_l(ω)
// =============================================================================
//
// The left Jacobian relates the derivative of the exponential map to
// perturbations in the Lie algebra:
//
//   exp(ω + δ) ≈ exp(J_l(ω) · δ) · exp(ω)
//
// or equivalently:
//   d/dt exp((ω + t·δ)) |_{t=0} = J_l(ω) · δ · exp(ω)    (in matrix form)
//
// Closed-form expression:
//
//   J_l(ω) = I + ((1 - cos θ) / θ²) [ω]× + ((θ - sin θ) / θ³) [ω]×²
//
// where θ = ‖ω‖.
//
// Taylor series for small θ:
//   (1 - cos θ) / θ² → 1/2 - θ²/24 + ...
//   (θ - sin θ) / θ³ → 1/6 - θ²/120 + ...
//
// So: J_l(ω) → I + (1/2)[ω]× + (1/6)[ω]×²  as θ → 0
//
// The LEFT Jacobian appears in the derivative of the exponential map:
//   ∂/∂ω exp([ω]×) · p = -[exp([ω]×)·p]× · J_l(ω)
//
// =============================================================================

inline Eigen::Matrix3d leftJacobianSO3(const Eigen::Vector3d& w) {
  double theta_sq = w.squaredNorm();
  double theta = std::sqrt(theta_sq);

  Eigen::Matrix3d W = hat(w);

  if (theta < 1e-10) {
    // J_l ≈ I + (1/2)[ω]× + (1/6)[ω]×²
    return Eigen::Matrix3d::Identity() + 0.5 * W;
  }

  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);

  double alpha = (1.0 - cos_theta) / theta_sq;
  double beta = (theta - sin_theta) / (theta_sq * theta);

  return Eigen::Matrix3d::Identity() + alpha * W + beta * (W * W);
}

// =============================================================================
// Inverse of the Left Jacobian: J_l^{-1}(ω)
// =============================================================================
//
// Used for converting between tangent-space perturbations and updates:
//
//   J_l^{-1}(ω) = I - (1/2)[ω]× + (1/θ² - (1+cosθ)/(2θ sinθ)) [ω]×²
//
// For small θ:
//   J_l^{-1}(ω) ≈ I - (1/2)[ω]× + (1/12)[ω]×²
//
// =============================================================================

inline Eigen::Matrix3d leftJacobianInverseSO3(const Eigen::Vector3d& w) {
  double theta_sq = w.squaredNorm();
  double theta = std::sqrt(theta_sq);

  Eigen::Matrix3d W = hat(w);

  if (theta < 1e-10) {
    // J_l^{-1} ≈ I - (1/2)[ω]× + (1/12)[ω]×²
    return Eigen::Matrix3d::Identity() - 0.5 * W;
  }

  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);

  // gamma = 1/θ² - (1 + cosθ)/(2θ sinθ)
  //
  // Alternative (more numerically stable): use the half-angle identity
  //   (1 + cosθ)/(2 sinθ) = cot(θ/2) / 2 = cos(θ/2)/(2 sin(θ/2))
  // So gamma = 1/θ² - cot(θ/2)/(2θ)
  double gamma = 1.0 / theta_sq - (1.0 + cos_theta) / (2.0 * theta * sin_theta);

  return Eigen::Matrix3d::Identity() - 0.5 * W + gamma * (W * W);
}

// =============================================================================
// Right Jacobian of SO(3): J_r(ω) = J_l(-ω)
// =============================================================================
//
// The right Jacobian satisfies:
//   exp(ω) · exp(δ) ≈ exp(ω + J_r(ω) · δ)
//
// Relation to left Jacobian:
//   J_r(ω) = J_l(-ω) = R(ω)^T · J_l(ω)
//
inline Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d& w) {
  return leftJacobianSO3(-w);
}

inline Eigen::Matrix3d rightJacobianInverseSO3(const Eigen::Vector3d& w) {
  return leftJacobianInverseSO3(-w);
}

}  // namespace lie
}  // namespace backend
}  // namespace substral
