#pragma once

#include <Eigen/Dense>

namespace substral {
namespace backend {
namespace ops {

// =============================================================================
// CPU Reference: Full Projection with Analytical Jacobians
// =============================================================================
//
// Mirrors the GPU implementation in project_with_jacobian_gpu.cuh exactly.
//
// Camera parameter layout (BAL format):
//   cam[0..2] = omega  (angle-axis rotation)
//   cam[3..5] = t  (translation)
//   cam[6]    = f  (focal length)
//   cam[7]    = k1 (radial distortion)
//   cam[8]    = k2 (radial distortion)
//
// All implementations are in project_with_jacobian_cpu.cpp.
// =============================================================================

// Stage 1: Rodrigues rotation with Jacobian
// P' = R(omega) . P
// Returns dP'/domega (3x3) and dP'/dP (3x3) = R(omega)
void rotatePointWithJacobianCPU(const Eigen::Vector3d& w,
                                const Eigen::Vector3d& P,
                                Eigen::Vector3d& P_out,
                                Eigen::Matrix3d& dPout_dw,
                                Eigen::Matrix3d& dPout_dP);

// Stage 3: Perspective divide with Jacobian
// (u, v) = (-X/Z, -Y/Z)
void perspectiveDivideWithJacobianCPU(const Eigen::Vector3d& P_cam,
                                      Eigen::Vector2d& uv,
                                      Eigen::Matrix<double, 2, 3>& duv_dPcam);

// Stage 4: Distortion + focal scaling with Jacobian
// (px, py) = f . D . (u, v)  where D = 1 + k1*r^2 + k2*r^4
void distortWithJacobianCPU(const Eigen::Vector2d& uv, double f, double k1,
                            double k2, Eigen::Vector2d& px_out,
                            Eigen::Matrix2d& dpx_duv,
                            Eigen::Matrix<double, 2, 3>& dpx_dfk1k2);

// Full fused projection with Jacobian (CPU)
//
// cam: 9 camera parameters
// pt:  3D point in world frame
//
// Outputs:
//   predicted: 2D projected pixel
//   J_cam:     2x9 Jacobian w.r.t. camera params (row-major)
//   J_pt:      2x3 Jacobian w.r.t. 3D point (row-major)
void projectWithJacobianCPU(const double* cam, const double* pt,
                            double* predicted, double* J_cam_out,
                            double* J_pt_out);

}  // namespace ops
}  // namespace backend
}  // namespace substral
