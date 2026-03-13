#include "subastral/backend/ops/project_with_jacobian_lie_cpu.hpp"

#include <Eigen/Dense>
#include <cmath>

#include "subastral/backend/lie/so3.hpp"
#include "subastral/backend/ops/project_with_jacobian_cpu.hpp"

namespace substral {
namespace backend {
namespace ops {

// =============================================================================
// Full fused projection with Lie group Jacobian (CPU)
// =============================================================================
//
// The projection pipeline is identical to the global parameterization:
//   1. P_rot = R(ω) · P_world
//   2. P_cam = P_rot + t
//   3. (u,v) = perspectiveDivide(P_cam)
//   4. (px,py) = distort(u, v, f, k1, k2)
//
// The difference is in the Jacobian computation for the pose part.
//
// For the Lie group parameterization, we use the left-perturbation model:
//   T_new = Exp(δξ) · T_old
//
// At δξ = 0:
//   P_cam(δξ) ≈ P_cam_0 - [P_rot]× · δφ + δρ
//
// So:
//   ∂P_cam/∂δφ = -[P_rot]×
//   ∂P_cam/∂δρ = I₃
//
// The chain rule through perspective+distortion gives us C = d(px,py)/dP_cam.
//
// Then:
//   J_cam cols 0-2 = C · (-[P_rot]×)
//   J_cam cols 3-5 = C
//   J_cam cols 6-8 = d(px,py)/d(f,k1,k2)
//
//   J_pt = C · R
//
// =============================================================================

void projectWithJacobianLieCPU(const double* cam, const double* pt,
                               double* predicted, double* J_cam_out,
                               double* J_pt_out) {
  Eigen::Map<const Eigen::Vector3d> w(cam);
  Eigen::Map<const Eigen::Vector3d> t(cam + 3);
  double f = cam[6], k1 = cam[7], k2 = cam[8];
  Eigen::Map<const Eigen::Vector3d> P_world(pt);

  // Stage 1: Rodrigues rotation
  // We need R and P_rot, but NOT dP_rot/dω (that's the old parameterization).
  // However, we still need R for J_pt = C · R.
  Eigen::Vector3d P_rot;
  Eigen::Matrix3d dProt_dw_unused, R;
  rotatePointWithJacobianCPU(w, P_world, P_rot, dProt_dw_unused, R);

  // Stage 2: Translation
  Eigen::Vector3d P_cam = P_rot + t;

  // Stage 3: Perspective divide
  Eigen::Vector2d uv;
  Eigen::Matrix<double, 2, 3> duv_dPcam;
  perspectiveDivideWithJacobianCPU(P_cam, uv, duv_dPcam);

  // Stage 4: Distortion + focal scaling
  Eigen::Vector2d px;
  Eigen::Matrix2d dpx_duv;
  Eigen::Matrix<double, 2, 3> dpx_dfk1k2;
  distortWithJacobianCPU(uv, f, k1, k2, px, dpx_duv, dpx_dfk1k2);

  // Output predicted pixel
  predicted[0] = px[0];
  predicted[1] = px[1];

  // ---- Chain rule ----
  // C = d(px,py)/dP_cam = dpx_duv (2×2) · duv_dPcam (2×3) → 2×3
  Eigen::Matrix<double, 2, 3> C = dpx_duv * duv_dPcam;

  // ---- Lie group Jacobian ----
  //
  // The left-perturbation Exp(δξ) · T gives:
  //   R_new = (I + [δφ]×) R
  //   t_new = t + δφ × t + δρ
  //
  // So P_cam_new = R_new · P + t_new
  //             = (I + [δφ]×) R·P + t + [δφ]× t + δρ
  //             = P_cam_0 + [δφ]× (R·P + t) + δρ
  //             = P_cam_0 + [δφ]× P_cam + δρ
  //             = P_cam_0 - [P_cam]× δφ + δρ
  //
  // Therefore: ∂P_cam/∂δφ = -[P_cam]×
  //
  Eigen::Matrix3d P_cam_cross = lie::hat(P_cam);

  // J_cam (2×9):
  //   cols 0-2: C · (-[P_cam]×)
  //   cols 3-5: C
  //   cols 6-8: dpx_dfk1k2
  Eigen::Matrix<double, 2, 9> J_cam;
  J_cam.block<2, 3>(0, 0) = -C * P_cam_cross;
  J_cam.block<2, 3>(0, 3) = C;
  J_cam.block<2, 3>(0, 6) = dpx_dfk1k2;

  // J_pt (2×3) = C · R
  Eigen::Matrix<double, 2, 3> J_pt = C * R;

  // Copy to output (row-major for consistency with GPU)
  Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J_cam_map(J_cam_out);
  J_cam_map = J_cam;

  Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_pt_map(J_pt_out);
  J_pt_map = J_pt;
}

}  // namespace ops
}  // namespace backend
}  // namespace substral
