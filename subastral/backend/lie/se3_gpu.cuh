#pragma once

#include <cuda_runtime.h>

#include "subastral/backend/lie/so3_gpu.cuh"

namespace substral {
namespace backend {
namespace lie {
namespace device {

// =============================================================================
// SE(3) Lie Group Operations — GPU Device Implementation
// =============================================================================
//
// Mirrors se3.hpp exactly, but uses raw double arrays and __device__ functions.
//
// Convention: ξ = (ω₀, ω₁, ω₂, v₀, v₁, v₂)
//   xi[0..2] = rotation part ω
//   xi[3..5] = translation part v
//
// SE(3) elements stored as double[16] in row-major 4×4:
//   T[0..8]  = top-left 3×3 rotation (row-major, indices 0,1,2,4,5,6,8,9,10)
//
// Actually, for simplicity we store the rotation R[9] and translation t[3]
// separately in the functions below, since the 4×4 matrix is rarely needed
// on GPU (we mostly need R and t for point transformation).
// =============================================================================

// =============================================================================
// exp: se(3) → SE(3)
// =============================================================================
//
// Input:  xi[6]  — twist vector (ω, v)
// Output: R[9]   — 3×3 rotation matrix (row-major)
//         t[3]   — translation vector
//
// R = exp([ω]×)
// t = J_l(ω) · v
//
__device__ inline void expSE3(const double* xi, double* R, double* t) {
  const double* w = xi;      // ω = xi[0..2]
  const double* v = xi + 3;  // v = xi[3..5]

  // Compute rotation
  expSO3(w, R);

  // Compute translation: t = J_l(ω) · v
  double Jl[9];
  leftJacobianSO3(w, Jl);
  mat3x3_vec(Jl, v, t);
}

// =============================================================================
// log: SE(3) → se(3)
// =============================================================================
//
// Input:  R[9]   — 3×3 rotation matrix (row-major)
//         t[3]   — translation vector
// Output: xi[6]  — twist vector (ω, v)
//
// ω = log_{SO(3)}(R)
// v = J_l(ω)^{-1} · t
//
__device__ inline void logSE3(const double* R, const double* t, double* xi) {
  // ω = logSO3(R)
  double* w = xi;       // xi[0..2]
  double* v = xi + 3;   // xi[3..5]

  logSO3(R, w);

  // v = J_l^{-1}(ω) · t
  double Jl_inv[9];
  leftJacobianInverseSO3(w, Jl_inv);
  mat3x3_vec(Jl_inv, t, v);
}

// =============================================================================
// SE(3) action on a point: p' = R · p + t
// =============================================================================
__device__ inline void transformPoint(const double* R, const double* t,
                                       const double* p, double* p_out) {
  mat3x3_vec(R, p, p_out);
  p_out[0] += t[0];
  p_out[1] += t[1];
  p_out[2] += t[2];
}

// =============================================================================
// SE(3) inverse: R_inv = R^T, t_inv = -R^T * t
// =============================================================================
__device__ inline void inverseSE3(const double* R, const double* t,
                                   double* R_inv, double* t_inv) {
  // R_inv = R^T (transpose)
  R_inv[0] = R[0]; R_inv[1] = R[3]; R_inv[2] = R[6];
  R_inv[3] = R[1]; R_inv[4] = R[4]; R_inv[5] = R[7];
  R_inv[6] = R[2]; R_inv[7] = R[5]; R_inv[8] = R[8];

  // t_inv = -R^T * t
  mat3x3_vec(R_inv, t, t_inv);
  t_inv[0] = -t_inv[0];
  t_inv[1] = -t_inv[1];
  t_inv[2] = -t_inv[2];
}

// =============================================================================
// Compose two SE(3) elements: (R1,t1) * (R2,t2) = (R1*R2, R1*t2 + t1)
// =============================================================================
__device__ inline void composeSE3(const double* R1, const double* t1,
                                   const double* R2, const double* t2,
                                   double* R_out, double* t_out) {
  mat3x3_mul(R1, R2, R_out);
  mat3x3_vec(R1, t2, t_out);
  t_out[0] += t1[0];
  t_out[1] += t1[1];
  t_out[2] += t1[2];
}

}  // namespace device
}  // namespace lie
}  // namespace backend
}  // namespace substral
