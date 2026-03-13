#pragma once

#include <cuda_runtime.h>

#include <cmath>

namespace substral {
namespace backend {
namespace lie {
namespace device {

// =============================================================================
// SO(3) Lie Group Operations — GPU Device Implementation
// =============================================================================
//
// Mirrors so3.hpp exactly, but uses raw double arrays instead of Eigen
// and is annotated with __device__ for use in CUDA kernels.
//
// All matrices are stored in ROW-MAJOR order as double[9] (3×3).
// All vectors are stored as double[3].
// =============================================================================

// =============================================================================
// hat: ℝ³ → so(3)  (skew-symmetric matrix, row-major)
// =============================================================================
__device__ inline void hat3(const double* w, double* W) {
  //  0   -w2   w1
  //  w2   0   -w0
  // -w1   w0   0
  W[0] = 0.0;   W[1] = -w[2];  W[2] = w[1];
  W[3] = w[2];  W[4] = 0.0;    W[5] = -w[0];
  W[6] = -w[1]; W[7] = w[0];   W[8] = 0.0;
}

// =============================================================================
// vee: so(3) → ℝ³
// =============================================================================
__device__ inline void vee3(const double* W, double* w) {
  w[0] = W[7];  // W(2,1)
  w[1] = W[2];  // W(0,2)
  w[2] = W[3];  // W(1,0)
}

// =============================================================================
// 3×3 matrix multiply: C = A * B  (all row-major)
// =============================================================================
__device__ inline void mat3x3_mul(const double* A, const double* B,
                                   double* C) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) {
        sum += A[i * 3 + k] * B[k * 3 + j];
      }
      C[i * 3 + j] = sum;
    }
  }
}

// =============================================================================
// 3×3 matrix-vector multiply: y = A * x  (A row-major)
// =============================================================================
__device__ inline void mat3x3_vec(const double* A, const double* x,
                                   double* y) {
  for (int i = 0; i < 3; ++i) {
    y[i] = A[i * 3 + 0] * x[0] + A[i * 3 + 1] * x[1] + A[i * 3 + 2] * x[2];
  }
}

// =============================================================================
// exp: so(3) → SO(3)  (Rodrigues formula)
// =============================================================================
//
// Input:  w[3]   — angle-axis vector ω
// Output: R[9]   — 3×3 rotation matrix (row-major)
//
// R = I + (sinθ/θ)[ω]× + ((1-cosθ)/θ²)[ω]×²
//
__device__ inline void expSO3(const double* w, double* R) {
  double theta_sq = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
  double theta = sqrt(theta_sq);

  double W[9];
  hat3(w, W);

  if (theta < 1e-10) {
    // R ≈ I + [ω]×
    for (int i = 0; i < 9; ++i) R[i] = W[i];
    R[0] += 1.0;
    R[4] += 1.0;
    R[8] += 1.0;
    return;
  }

  double sin_theta = sin(theta);
  double cos_theta = cos(theta);
  double a = sin_theta / theta;
  double b = (1.0 - cos_theta) / theta_sq;

  double WW[9];
  mat3x3_mul(W, W, WW);

  // R = I + a*W + b*W*W
  for (int i = 0; i < 9; ++i) {
    R[i] = a * W[i] + b * WW[i];
  }
  R[0] += 1.0;
  R[4] += 1.0;
  R[8] += 1.0;
}

// =============================================================================
// log: SO(3) → so(3)
// =============================================================================
//
// Input:  R[9]   — 3×3 rotation matrix (row-major)
// Output: w[3]   — angle-axis vector ω
//
__device__ inline void logSO3(const double* R, double* w) {
  double trace = R[0] + R[4] + R[8];
  double cos_theta = (trace - 1.0) / 2.0;

  // Clamp
  if (cos_theta > 1.0) cos_theta = 1.0;
  if (cos_theta < -1.0) cos_theta = -1.0;
  double theta = acos(cos_theta);

  // Case 1: θ ≈ 0
  if (theta < 1e-10) {
    // [ω]× ≈ (R - R^T)/2
    // w = vee((R - R^T)/2)
    w[0] = (R[7] - R[5]) / 2.0;  // (R(2,1) - R(1,2))/2
    w[1] = (R[2] - R[6]) / 2.0;  // (R(0,2) - R(2,0))/2
    w[2] = (R[3] - R[1]) / 2.0;  // (R(1,0) - R(0,1))/2
    return;
  }

  // Case 2: θ ≈ π
  if (theta > M_PI - 1e-6) {
    // Find column of (R + I) with largest norm
    // R+I diagonal: R[0]+1, R[4]+1, R[8]+1
    double col_norms[3];
    col_norms[0] = (R[0] + 1.0) * (R[0] + 1.0) + (R[3] + 0.0) * R[3] +
                   (R[6] + 0.0) * R[6];
    // Actually: col j of (R+I) = (R[0*3+j]+delta(0,j), R[1*3+j]+delta(1,j),
    // R[2*3+j]+delta(2,j))
    col_norms[0] =
        (R[0] + 1) * (R[0] + 1) + R[3] * R[3] + R[6] * R[6];
    col_norms[1] =
        R[1] * R[1] + (R[4] + 1) * (R[4] + 1) + R[7] * R[7];
    col_norms[2] =
        R[2] * R[2] + R[5] * R[5] + (R[8] + 1) * (R[8] + 1);

    int best_col = 0;
    if (col_norms[1] > col_norms[best_col]) best_col = 1;
    if (col_norms[2] > col_norms[best_col]) best_col = 2;

    // Extract column best_col of (R + I)
    double k[3];
    k[0] = R[0 * 3 + best_col] + (best_col == 0 ? 1.0 : 0.0);
    k[1] = R[1 * 3 + best_col] + (best_col == 1 ? 1.0 : 0.0);
    k[2] = R[2 * 3 + best_col] + (best_col == 2 ? 1.0 : 0.0);

    // Normalize
    double norm = sqrt(k[0] * k[0] + k[1] * k[1] + k[2] * k[2]);
    if (norm > 1e-12) {
      k[0] /= norm;
      k[1] /= norm;
      k[2] /= norm;
    }

    w[0] = theta * k[0];
    w[1] = theta * k[1];
    w[2] = theta * k[2];
    return;
  }

  // Case 3: General case
  double factor = theta / (2.0 * sin(theta));
  w[0] = factor * (R[7] - R[5]);  // (R(2,1) - R(1,2))
  w[1] = factor * (R[2] - R[6]);  // (R(0,2) - R(2,0))
  w[2] = factor * (R[3] - R[1]);  // (R(1,0) - R(0,1))
}

// =============================================================================
// Left Jacobian of SO(3): J_l(ω)
// =============================================================================
//
// Input:  w[3]   — angle-axis vector ω
// Output: Jl[9]  — 3×3 left Jacobian (row-major)
//
// J_l(ω) = I + ((1-cosθ)/θ²)[ω]× + ((θ-sinθ)/θ³)[ω]×²
//
__device__ inline void leftJacobianSO3(const double* w, double* Jl) {
  double theta_sq = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
  double theta = sqrt(theta_sq);

  double W[9];
  hat3(w, W);

  if (theta < 1e-10) {
    // J_l ≈ I + (1/2)[ω]×
    for (int i = 0; i < 9; ++i) Jl[i] = 0.5 * W[i];
    Jl[0] += 1.0;
    Jl[4] += 1.0;
    Jl[8] += 1.0;
    return;
  }

  double sin_theta = sin(theta);
  double cos_theta = cos(theta);

  double alpha = (1.0 - cos_theta) / theta_sq;
  double beta = (theta - sin_theta) / (theta_sq * theta);

  double WW[9];
  mat3x3_mul(W, W, WW);

  // J_l = I + alpha*W + beta*W*W
  for (int i = 0; i < 9; ++i) {
    Jl[i] = alpha * W[i] + beta * WW[i];
  }
  Jl[0] += 1.0;
  Jl[4] += 1.0;
  Jl[8] += 1.0;
}

// =============================================================================
// Inverse of Left Jacobian: J_l^{-1}(ω)
// =============================================================================
//
// Input:  w[3]      — angle-axis vector ω
// Output: Jl_inv[9] — 3×3 inverse left Jacobian (row-major)
//
__device__ inline void leftJacobianInverseSO3(const double* w,
                                               double* Jl_inv) {
  double theta_sq = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
  double theta = sqrt(theta_sq);

  double W[9];
  hat3(w, W);

  if (theta < 1e-10) {
    // J_l^{-1} ≈ I - (1/2)[ω]×
    for (int i = 0; i < 9; ++i) Jl_inv[i] = -0.5 * W[i];
    Jl_inv[0] += 1.0;
    Jl_inv[4] += 1.0;
    Jl_inv[8] += 1.0;
    return;
  }

  double sin_theta = sin(theta);
  double cos_theta = cos(theta);

  double gamma = 1.0 / theta_sq -
                 (1.0 + cos_theta) / (2.0 * theta * sin_theta);

  double WW[9];
  mat3x3_mul(W, W, WW);

  // J_l^{-1} = I - 0.5*W + gamma*W*W
  for (int i = 0; i < 9; ++i) {
    Jl_inv[i] = -0.5 * W[i] + gamma * WW[i];
  }
  Jl_inv[0] += 1.0;
  Jl_inv[4] += 1.0;
  Jl_inv[8] += 1.0;
}

}  // namespace device
}  // namespace lie
}  // namespace backend
}  // namespace substral
