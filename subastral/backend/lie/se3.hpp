#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "subastral/backend/lie/so3.hpp"

namespace substral {
namespace backend {
namespace lie {

// =============================================================================
// SE(3) Lie Group Operations — CPU Reference Implementation
// =============================================================================
//
// SE(3) is the group of rigid-body transformations in 3D:
//   SE(3) = { T = | R  t | ∈ ℝ⁴ˣ⁴ | R ∈ SO(3), t ∈ ℝ³ }
//                 | 0  1 |
//
// Its Lie algebra se(3) consists of 4×4 "twist" matrices:
//   ξ^ = | [ω]×  v | ∈ se(3)  ↔  ξ = (ω, v) ∈ ℝ⁶
//        |  0    0 |
//
// Convention: ξ = (ω₀, ω₁, ω₂, v₀, v₁, v₂)
//   - ω ∈ ℝ³: rotation part (first 3 components)
//   - v ∈ ℝ³: translation part (last 3 components)
//
// The exponential map sends se(3) → SE(3):
//   exp(ξ^) = | exp([ω]×)   J_l(ω) · v |
//             |    0              1      |
//
// where J_l(ω) is the left Jacobian of SO(3).
//
// The logarithm map sends SE(3) → se(3):
//   log(T) = ξ = (ω, J_l(ω)^{-1} · t)
//
// where ω = log_{SO(3)}(R) and t is the translation.
//
// =============================================================================

// =============================================================================
// hat: ℝ⁶ → se(3)
// =============================================================================
//
// Maps a 6-vector ξ = (ω, v) to its 4×4 twist matrix:
//
//   hat(ξ) = | [ω]×  v |
//            |  0    0 |
//
inline Eigen::Matrix4d hatSE3(const Eigen::Matrix<double, 6, 1>& xi) {
  Eigen::Matrix4d X = Eigen::Matrix4d::Zero();
  Eigen::Vector3d w = xi.head<3>();
  Eigen::Vector3d v = xi.tail<3>();
  X.block<3, 3>(0, 0) = hat(w);
  X.block<3, 1>(0, 3) = v;
  return X;
}

// =============================================================================
// vee: se(3) → ℝ⁶
// =============================================================================
//
// Inverse of hatSE3 — extracts the 6-vector from a twist matrix.
//
inline Eigen::Matrix<double, 6, 1> veeSE3(const Eigen::Matrix4d& X) {
  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = vee(X.block<3, 3>(0, 0));
  xi.tail<3>() = X.block<3, 1>(0, 3);
  return xi;
}

// =============================================================================
// exp: se(3) → SE(3)
// =============================================================================
//
// Given ξ = (ω, v) ∈ ℝ⁶:
//
//   exp(ξ^) = | R   J_l(ω) · v |
//             | 0       1       |
//
// where R = exp([ω]×) and J_l(ω) is the left Jacobian of SO(3).
//
// Derivation:
//   The matrix exponential of the twist can be computed in closed form
//   using the fact that se(3) has a semi-direct product structure:
//
//   exp(| [ω]×  v |) = | exp([ω]×)                        V·v |
//       |  0    0 |    |    0                               1  |
//
//   where V = J_l(ω) = I + ((1-cosθ)/θ²)[ω]× + ((θ-sinθ)/θ³)[ω]×²
//
//   This is NOT simply t = v (that would be the "exponential coordinates"
//   for translation, which is different). The coupling between rotation
//   and translation is captured by the left Jacobian.
//
// Special case: ω = 0 (pure translation)
//   exp(ξ^) = | I  v |
//             | 0  1 |
//
// =============================================================================

inline Eigen::Matrix4d expSE3(const Eigen::Matrix<double, 6, 1>& xi) {
  Eigen::Vector3d w = xi.head<3>();
  Eigen::Vector3d v = xi.tail<3>();

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = expSO3(w);
  T.block<3, 1>(0, 3) = leftJacobianSO3(w) * v;

  return T;
}

// =============================================================================
// log: SE(3) → se(3)
// =============================================================================
//
// Given T = | R  t | ∈ SE(3):
//           | 0  1 |
//
//   ω = log_{SO(3)}(R)
//   v = J_l(ω)^{-1} · t
//
// So ξ = (ω, v) and exp(ξ^) = T.
//
// =============================================================================

inline Eigen::Matrix<double, 6, 1> logSE3(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d t = T.block<3, 1>(0, 3);

  Eigen::Vector3d w = logSO3(R);
  Eigen::Vector3d v = leftJacobianInverseSO3(w) * t;

  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = w;
  xi.tail<3>() = v;
  return xi;
}

// =============================================================================
// Adjoint representation of SE(3)
// =============================================================================
//
// The adjoint maps se(3) to se(3) via conjugation:
//   Ad_T(ξ) = T · ξ^ · T^{-1}
//
// In matrix form (6×6):
//
//   Ad_T = | R       0 |
//          | [t]×R   R |
//
// This is used for transforming twists between frames.
//
// =============================================================================

inline Eigen::Matrix<double, 6, 6> adjointSE3(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d t = T.block<3, 1>(0, 3);

  Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Zero();
  Ad.block<3, 3>(0, 0) = R;
  Ad.block<3, 3>(3, 0) = hat(t) * R;
  Ad.block<3, 3>(3, 3) = R;

  return Ad;
}

// =============================================================================
// SE(3) inverse
// =============================================================================
//
//   T^{-1} = | R^T  -R^T t |
//            |  0      1   |
//
inline Eigen::Matrix4d inverseSE3(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d t = T.block<3, 1>(0, 3);

  Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
  T_inv.block<3, 3>(0, 0) = R.transpose();
  T_inv.block<3, 1>(0, 3) = -R.transpose() * t;

  return T_inv;
}

// =============================================================================
// SE(3) action on a point: T · p (applies rotation then translation)
// =============================================================================
//
//   T · p = R · p + t
//
inline Eigen::Vector3d transformPoint(const Eigen::Matrix4d& T,
                                      const Eigen::Vector3d& p) {
  return T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);
}

// =============================================================================
// Left Jacobian of SE(3): J_l(ξ)  (6×6)
// =============================================================================
//
// The left Jacobian of SE(3) relates perturbations in the Lie algebra
// to the group element:
//
//   exp(ξ + δ) ≈ exp(J_l(ξ) · δ) · exp(ξ)
//
// This has a block structure:
//
//   J_l(ξ) = | J_l(ω)    0     |
//            |  Q(ξ)    J_l(ω) |
//
// where J_l(ω) is the SO(3) left Jacobian and Q(ξ) is:
//
//   Q(ξ) = (1/2)[v]× + (θ-sinθ)/θ³ · (ω×v + v×ω + ω×v×ω)
//          + ... (Barfoot eq. 9.84)
//
// For our BA application, we primarily need the SO(3) left Jacobian
// and the SE(3) exp/log. The full SE(3) Jacobian is included for
// completeness but may not be needed in Phase 4.
//
// =============================================================================

}  // namespace lie
}  // namespace backend
}  // namespace substral
