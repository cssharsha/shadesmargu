#pragma once

#include <cuda_runtime.h>

#include "subastral/backend/ops/distortion_gpu.cuh"
#include "subastral/backend/ops/perspective_gpu.cuh"
#include "subastral/backend/ops/rodrigues_gpu.cuh"

namespace substral {
namespace backend {
namespace ops {
namespace device {

// =============================================================================
// Full Projection with Analytical Jacobians — Fused Kernel
// =============================================================================
//
// Composes the three stages of the BAL projection pipeline and applies the
// chain rule to produce the full Jacobians:
//
//   J_cam: d(px,py)/d(ω₁,ω₂,ω₃, t₁,t₂,t₃, f, k1, k2)  — 2×9
//   J_pt:  d(px,py)/d(X, Y, Z)                             — 2×3
//
// Camera parameter layout (BAL format):
//   cam[0..2] = ω  (angle-axis rotation)
//   cam[3..5] = t  (translation)
//   cam[6]    = f  (focal length)
//   cam[7]    = k1 (radial distortion)
//   cam[8]    = k2 (radial distortion)
//
// =============================================================================
// Chain Rule Structure
// =============================================================================
//
// The pipeline is:
//
//   P_world --[R(ω)]--> P_rot --[+t]--> P_cam --[persp]--> (u,v) --[dist]--> (px,py)
//
// Stage 1: P_rot = R(ω) · P_world
//   Outputs: dP_rot/dω (3×3),  dP_rot/dP_world (3×3) = R
//
// Stage 2: P_cam = P_rot + t
//   dP_cam/dP_rot = I₃
//   dP_cam/dt     = I₃
//   So: dP_cam/dω = dP_rot/dω,  dP_cam/dP_world = R,  dP_cam/dt = I₃
//
// Stage 3: (u,v) = perspectiveDivide(P_cam)
//   Outputs: d(u,v)/dP_cam (2×3)
//
// Stage 4: (px,py) = distort(u, v, f, k1, k2)
//   Outputs: d(px,py)/d(u,v) (2×2),  d(px,py)/d(f,k1,k2) (2×3)
//
// Now apply the chain rule:
//
//   d(px,py)/d(u,v) is 2×2, call it A
//   d(u,v)/dP_cam   is 2×3, call it B
//
//   C = A · B  is 2×3: d(px,py)/dP_cam
//
// Then:
//   d(px,py)/dω  = C · dP_cam/dω  = C · dP_rot/dω     (2×3 · 3×3 = 2×3)
//   d(px,py)/dt  = C · I₃          = C                  (2×3)
//   d(px,py)/df, dk1, dk2  come directly from stage 4   (2×3)
//   d(px,py)/dP_world = C · R                            (2×3 · 3×3 = 2×3)
//
// Final Jacobian assembly (J_cam is 2×9, J_pt is 2×3):
//
//   J_cam = [ C·dP_rot/dω | C | d(px,py)/d(f,k1,k2) ]
//             columns 0-2   3-5       6-8
//
//   J_pt  = C · R
//
// =============================================================================

// Helper: 2×3 = 2×2 · 2×3  (row-major)
__device__ inline void matmul_2x2_2x3(const double* A, const double* B,
                                       double* C) {
  // C[0] = A[0]*B[0] + A[1]*B[3]
  // C[1] = A[0]*B[1] + A[1]*B[4]
  // C[2] = A[0]*B[2] + A[1]*B[5]
  // C[3] = A[2]*B[0] + A[3]*B[3]
  // C[4] = A[2]*B[1] + A[3]*B[4]
  // C[5] = A[2]*B[2] + A[3]*B[5]
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      C[i * 3 + j] = A[i * 2 + 0] * B[0 * 3 + j] +
                      A[i * 2 + 1] * B[1 * 3 + j];
    }
  }
}

// Helper: 2×3 = 2×3 · 3×3  (row-major)
__device__ inline void matmul_2x3_3x3(const double* A, const double* B,
                                       double* C) {
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      C[i * 3 + j] = A[i * 3 + 0] * B[0 * 3 + j] +
                      A[i * 3 + 1] * B[1 * 3 + j] +
                      A[i * 3 + 2] * B[2 * 3 + j];
    }
  }
}

__device__ inline void projectWithJacobian(
    const double* cam,      // [9] camera parameters
    const double* pt,       // [3] 3D point (world frame)
    double* predicted,      // [2] projected pixel coordinates
    double* J_cam,          // [18] row-major 2×9: d(px,py)/d(cam params)
    double* J_pt            // [6]  row-major 2×3: d(px,py)/d(point)
) {
  // ---- Stage 1: Rodrigues rotation ----
  // P_rot = R(ω) · P_world
  const double* w = cam;       // ω = cam[0..2]
  double P_rot[3];
  double dProt_dw[9];           // 3×3 row-major
  double dProt_dP[9];           // 3×3 row-major = R(ω)
  rotatePointWithJacobian(w, pt, P_rot, dProt_dw, dProt_dP);

  // ---- Stage 2: Translation ----
  // P_cam = P_rot + t
  const double* t = cam + 3;   // t = cam[3..5]
  double P_cam[3] = {P_rot[0] + t[0], P_rot[1] + t[1], P_rot[2] + t[2]};
  // dP_cam/dω = dP_rot/dω  (translation doesn't depend on ω)
  // dP_cam/dt = I₃
  // dP_cam/dP_world = dP_rot/dP = R

  // ---- Stage 3: Perspective divide ----
  // (u, v) = perspectiveDivide(P_cam)
  double uv[2];
  double duv_dPcam[6];         // 2×3 row-major
  perspectiveDivideWithJacobian(P_cam, uv, duv_dPcam);

  // ---- Stage 4: Distortion + focal scaling ----
  // (px, py) = distort(u, v, f, k1, k2)
  double f  = cam[6];
  double k1 = cam[7];
  double k2 = cam[8];

  double dpx_duv[4];           // 2×2 row-major: d(px,py)/d(u,v)
  double dpx_dfk1k2[6];        // 2×3 row-major: d(px,py)/d(f,k1,k2)
  distortWithJacobian(uv, f, k1, k2, predicted, dpx_duv, dpx_dfk1k2);

  // ---- Chain rule: compose Jacobians ----

  // C = d(px,py)/dP_cam = d(px,py)/d(u,v) · d(u,v)/dP_cam
  //   = dpx_duv (2×2) · duv_dPcam (2×3) → C (2×3)
  double C[6];
  matmul_2x2_2x3(dpx_duv, duv_dPcam, C);

  // J_cam columns 0-2: d(px,py)/dω = C · dP_rot/dω
  //   = C (2×3) · dProt_dw (3×3) → 2×3
  double dPx_dw[6];
  matmul_2x3_3x3(C, dProt_dw, dPx_dw);

  J_cam[0] = dPx_dw[0];  J_cam[1] = dPx_dw[1];  J_cam[2] = dPx_dw[2];
  J_cam[9] = dPx_dw[3];  J_cam[10] = dPx_dw[4]; J_cam[11] = dPx_dw[5];

  // J_cam columns 3-5: d(px,py)/dt = C · I₃ = C
  J_cam[3] = C[0];  J_cam[4] = C[1];  J_cam[5] = C[2];
  J_cam[12] = C[3]; J_cam[13] = C[4]; J_cam[14] = C[5];

  // J_cam columns 6-8: d(px,py)/d(f, k1, k2)
  J_cam[6] = dpx_dfk1k2[0];  J_cam[7] = dpx_dfk1k2[1];  J_cam[8] = dpx_dfk1k2[2];
  J_cam[15] = dpx_dfk1k2[3]; J_cam[16] = dpx_dfk1k2[4]; J_cam[17] = dpx_dfk1k2[5];

  // J_pt: d(px,py)/dP_world = C · R = C · dProt_dP
  matmul_2x3_3x3(C, dProt_dP, J_pt);
}

}  // namespace device
}  // namespace ops
}  // namespace backend
}  // namespace substral
