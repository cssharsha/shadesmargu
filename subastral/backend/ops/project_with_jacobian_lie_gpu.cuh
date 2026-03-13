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
// Full Projection with Lie Group Jacobians — GPU Fused Kernel
// =============================================================================
//
// SE(3) Lie group version of projectWithJacobian.
//
// The Jacobian J_cam (2×9) uses left-perturbation on SE(3):
//   cols 0-2: ∂(px,py)/∂δφ  (rotation perturbation)
//   cols 3-5: ∂(px,py)/∂δρ  (translation perturbation)
//   cols 6-8: ∂(px,py)/∂(f, k1, k2)  (intrinsics, additive)
//
// Derivation (see project_with_jacobian_lie_cpu.hpp for full details):
//
//   T_new = Exp(δξ) · T_old  where δξ = (δφ, δρ) ∈ se(3)
//
//   P_cam_new = P_cam_0 - [P_cam]× · δφ + δρ   (to first order)
//
//   ∂P_cam/∂δφ = -[P_cam]×
//   ∂P_cam/∂δρ = I₃
//
// This is SIMPLER than the global angle-axis Jacobian because we don't
// need the complex Rodrigues derivative dP_rot/dω. We only need:
//   - The rotation matrix R (for computing P_rot and for J_pt)
//   - P_cam = R·P + t  (for the skew-symmetric matrix)
//   - The perspective+distortion chain C = d(px,py)/dP_cam
//
// =============================================================================

__device__ inline void projectWithJacobianLie(
    const double* cam,      // [9] camera parameters
    const double* pt,       // [3] 3D point (world frame)
    double* predicted,      // [2] projected pixel coordinates
    double* J_cam,          // [18] row-major 2×9: d(px,py)/d(δφ,δρ,δf,δk1,δk2)
    double* J_pt            // [6]  row-major 2×3: d(px,py)/d(point)
) {
  // ---- Stage 1: Rodrigues rotation ----
  // We need R and P_rot but NOT dP_rot/dω (that's the old parameterization).
  // We use rotatePointWithJacobian to get R = dProt_dP, but discard dProt_dw.
  const double* w = cam;       // ω = cam[0..2]
  double P_rot[3];
  double dProt_dw_unused[9];   // 3×3 — not needed for Lie Jacobian
  double R[9];                 // 3×3 row-major = R(ω)
  rotatePointWithJacobian(w, pt, P_rot, dProt_dw_unused, R);

  // ---- Stage 2: Translation ----
  const double* t = cam + 3;   // t = cam[3..5]
  double P_cam[3] = {P_rot[0] + t[0], P_rot[1] + t[1], P_rot[2] + t[2]};

  // ---- Stage 3: Perspective divide ----
  double uv[2];
  double duv_dPcam[6];         // 2×3 row-major
  perspectiveDivideWithJacobian(P_cam, uv, duv_dPcam);

  // ---- Stage 4: Distortion + focal scaling ----
  double f  = cam[6];
  double k1 = cam[7];
  double k2 = cam[8];

  double dpx_duv[4];           // 2×2 row-major
  double dpx_dfk1k2[6];        // 2×3 row-major
  distortWithJacobian(uv, f, k1, k2, predicted, dpx_duv, dpx_dfk1k2);

  // ---- Chain rule ----
  // C = d(px,py)/dP_cam = dpx_duv (2×2) · duv_dPcam (2×3) → C (2×3)
  double C[6];
  matmul_2x2_2x3(dpx_duv, duv_dPcam, C);

  // ---- Lie group Jacobian: cols 0-2 ----
  //
  // ∂P_cam/∂δφ = -[P_cam]×
  //
  // -[P_cam]× = |  0        P_cam[2]  -P_cam[1] |
  //             | -P_cam[2]  0         P_cam[0]  |
  //             |  P_cam[1] -P_cam[0]  0         |
  //
  // J_cam cols 0-2 = C · (-[P_cam]×) = C (2×3) · (-[P_cam]×) (3×3) → 2×3
  //
  // We compute this inline without forming the full 3×3 matrix:
  //   (-[P_cam]×) column j:
  //     col 0: (0, -P_cam[2], P_cam[1])
  //     col 1: (P_cam[2], 0, -P_cam[0])
  //     col 2: (-P_cam[1], P_cam[0], 0)
  //
  double neg_skew[9];  // -[P_cam]× row-major
  neg_skew[0] = 0.0;        neg_skew[1] = P_cam[2];   neg_skew[2] = -P_cam[1];
  neg_skew[3] = -P_cam[2];  neg_skew[4] = 0.0;        neg_skew[5] = P_cam[0];
  neg_skew[6] = P_cam[1];   neg_skew[7] = -P_cam[0];  neg_skew[8] = 0.0;

  double dPx_dphi[6];  // 2×3
  matmul_2x3_3x3(C, neg_skew, dPx_dphi);

  J_cam[0] = dPx_dphi[0];  J_cam[1] = dPx_dphi[1];  J_cam[2] = dPx_dphi[2];
  J_cam[9] = dPx_dphi[3];  J_cam[10] = dPx_dphi[4]; J_cam[11] = dPx_dphi[5];

  // ---- Lie group Jacobian: cols 3-5 ----
  // ∂P_cam/∂δρ = I₃, so J_cam cols 3-5 = C
  J_cam[3] = C[0];  J_cam[4] = C[1];  J_cam[5] = C[2];
  J_cam[12] = C[3]; J_cam[13] = C[4]; J_cam[14] = C[5];

  // ---- Intrinsics: cols 6-8 ----
  J_cam[6] = dpx_dfk1k2[0];  J_cam[7] = dpx_dfk1k2[1];  J_cam[8] = dpx_dfk1k2[2];
  J_cam[15] = dpx_dfk1k2[3]; J_cam[16] = dpx_dfk1k2[4]; J_cam[17] = dpx_dfk1k2[5];

  // ---- J_pt: d(px,py)/dP_world = C · R ----
  matmul_2x3_3x3(C, R, J_pt);
}

}  // namespace device
}  // namespace ops
}  // namespace backend
}  // namespace substral
