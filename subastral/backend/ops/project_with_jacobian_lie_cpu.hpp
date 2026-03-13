#pragma once

#include <Eigen/Dense>

namespace substral {
namespace backend {
namespace ops {

// =============================================================================
// CPU Reference: Full Projection with Lie Group Jacobians
// =============================================================================
//
// This is the SE(3) Lie group version of projectWithJacobianCPU.
//
// Camera parameter layout (same as BAL format):
//   cam[0..2] = ω  (angle-axis rotation, SO(3) log coordinates)
//   cam[3..5] = t  (translation)
//   cam[6]    = f  (focal length)
//   cam[7]    = k1 (radial distortion)
//   cam[8]    = k2 (radial distortion)
//
// The key difference from the global parameterization:
//
//   OLD: J_cam columns 0-2 = ∂(px,py)/∂ω  (global angle-axis derivative)
//   NEW: J_cam columns 0-2 = ∂(px,py)/∂δφ (left-perturbation on SO(3))
//        J_cam columns 3-5 = ∂(px,py)/∂δρ (left-perturbation on translation)
//
// The perturbation model is:
//   T_new = Exp(δξ) · T_old
//
// where δξ = (δφ, δρ) ∈ se(3), with δφ ∈ ℝ³ (rotation) and δρ ∈ ℝ³ (trans).
//
// At δξ = 0, the perturbed camera frame transforms a world point as:
//   R_new = (I + [δφ]×) R,  t_new = t + δφ × t + δρ
//   P_cam(δξ) = R_new · P_world + t_new
//             = (I + [δφ]×) R · P + t + [δφ]× t + δρ
//             = P_cam_0 + [δφ]× (R·P + t) + δρ
//             = P_cam_0 + [δφ]× P_cam_0 + δρ
//             = P_cam_0 - [P_cam_0]× · δφ + δρ
//
// where P_cam_0 = R · P_world + t.
//
// Therefore:
//   ∂P_cam/∂δφ = -[P_cam]×    (3×3 skew-symmetric)
//   ∂P_cam/∂δρ = I₃           (3×3 identity)
//
// The full chain rule gives:
//   J_cam (2×9):
//     cols 0-2: C · (-[P_cam]×)   where C = d(px,py)/dP_cam
//     cols 3-5: C                  (same as before)
//     cols 6-8: d(px,py)/d(f,k1,k2)  (same as before)
//
//   J_pt (2×3): C · R            (same as before)
//
// NOTE: J_cam is still 2×9. The first 6 columns now represent the se(3)
// perturbation (δφ, δρ) instead of the global (δω, δt). The last 3 columns
// (intrinsics) are unchanged.
//
// =============================================================================

// Full fused projection with Lie group Jacobian (CPU)
//
// cam: 9 camera parameters
// pt:  3D point in world frame
//
// Outputs:
//   predicted: 2D projected pixel
//   J_cam:     2x9 Jacobian w.r.t. (δφ, δρ, δf, δk1, δk2) (row-major)
//   J_pt:      2x3 Jacobian w.r.t. 3D point (row-major)
void projectWithJacobianLieCPU(const double* cam, const double* pt,
                               double* predicted, double* J_cam_out,
                               double* J_pt_out);

}  // namespace ops
}  // namespace backend
}  // namespace substral
