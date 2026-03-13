#include "subastral/backend/ops/project_with_jacobian_cpu.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace substral {
namespace backend {
namespace ops {

// =============================================================================
// Stage 1: Rodrigues rotation with Jacobian
// =============================================================================
//
// Rodrigues formula:
//   P' = P cos(theta) + (k x P) sin(theta) + k (k . P)(1 - cos(theta))
//
// where k = omega / theta, theta = ||omega||.
//
// For the Jacobian dP'/domega, we use the decomposition:
//   R(omega) = I + a [omega]x + b [omega]x^2
// where a = sin(theta)/theta, b = (1 - cos(theta))/theta^2.
//
// Then dP'/domega_j = da*w[j]*(omega x P) + a*(e_j x P)
//                   + db*w[j]*(omega x (omega x P))
//                   + b*(e_j x (omega x P) + omega x (e_j x P))
//
// where da = (cos(theta) - a) / theta^2
//       db = (a - 2b) / theta^2
//
// =============================================================================

void rotatePointWithJacobianCPU(const Eigen::Vector3d& w,
                                const Eigen::Vector3d& P,
                                Eigen::Vector3d& P_out,
                                Eigen::Matrix3d& dPout_dw,
                                Eigen::Matrix3d& dPout_dP) {
  double theta_sq = w.squaredNorm();
  double theta = std::sqrt(theta_sq);

  if (theta < 1e-10) {
    // Small angle: P' = P + omega x P
    P_out = P + w.cross(P);

    // dP'/domega = -[P]x
    dPout_dw << 0, P[2], -P[1], -P[2], 0, P[0], P[1], -P[0], 0;

    // dP'/dP = I + [omega]x
    Eigen::Matrix3d wx;
    wx << 0, -w[2], w[1], w[2], 0, -w[0], -w[1], w[0], 0;
    dPout_dP = Eigen::Matrix3d::Identity() + wx;
    return;
  }

  Eigen::Vector3d k = w / theta;
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);
  double one_minus_cos = 1.0 - cos_theta;

  // Rodrigues formula
  double k_dot_P = k.dot(P);
  Eigen::Vector3d k_cross_P = k.cross(P);
  P_out = P * cos_theta + k_cross_P * sin_theta + k * k_dot_P * one_minus_cos;

  // ---- dP'/domega ----
  double a = sin_theta / theta;
  double b = one_minus_cos / theta_sq;
  double da = (cos_theta - a) / theta_sq;
  double db = (a - 2.0 * b) / theta_sq;

  Eigen::Vector3d wxP = w.cross(P);
  Eigen::Vector3d wwxP = w.cross(wxP);

  for (int j = 0; j < 3; ++j) {
    Eigen::Vector3d ej = Eigen::Vector3d::Zero();
    ej[j] = 1.0;

    Eigen::Vector3d ejxP = ej.cross(P);
    Eigen::Vector3d ejxwxP = ej.cross(wxP);
    Eigen::Vector3d wxejxP = w.cross(ejxP);

    Eigen::Vector3d col =
        da * w[j] * wxP + a * ejxP + db * w[j] * wwxP + b * (ejxwxP + wxejxP);

    dPout_dw.col(j) = col;
  }

  // ---- dP'/dP = R(omega) = I + a*[omega]x + b*[omega]x^2 ----
  Eigen::Matrix3d wx;
  wx << 0, -w[2], w[1], w[2], 0, -w[0], -w[1], w[0], 0;
  dPout_dP = Eigen::Matrix3d::Identity() + a * wx + b * (wx * wx);
}

// =============================================================================
// Stage 3: Perspective divide with Jacobian
// =============================================================================
//
// (u, v) = (-X/Z, -Y/Z)
//
// Jacobian:
//   du/dX = -1/Z,  du/dY = 0,     du/dZ = X/Z^2
//   dv/dX = 0,     dv/dY = -1/Z,  dv/dZ = Y/Z^2
//
// =============================================================================

void perspectiveDivideWithJacobianCPU(const Eigen::Vector3d& P_cam,
                                      Eigen::Vector2d& uv,
                                      Eigen::Matrix<double, 2, 3>& duv_dPcam) {
  double X = P_cam[0], Y = P_cam[1], Z = P_cam[2];
  double inv_Z = 1.0 / Z;
  double inv_Z2 = inv_Z * inv_Z;

  uv[0] = -X * inv_Z;
  uv[1] = -Y * inv_Z;

  duv_dPcam << -inv_Z, 0.0, X * inv_Z2, 0.0, -inv_Z, Y * inv_Z2;
}

// =============================================================================
// Stage 4: Distortion + focal scaling with Jacobian
// =============================================================================
//
// D = 1 + k1*r^2 + k2*r^4    where r^2 = u^2 + v^2
// (px, py) = f * D * (u, v)
//
// Jacobian w.r.t. (u,v):
//   S = k1 + 2*k2*r^2
//   dpx/du = f * (2*u^2*S + D)
//   dpx/dv = f * 2*u*v*S
//   dpy/du = f * 2*u*v*S
//   dpy/dv = f * (2*v^2*S + D)
//
// Jacobian w.r.t. (f, k1, k2):
//   dpx/df  = D*u,     dpy/df  = D*v
//   dpx/dk1 = f*r^2*u, dpy/dk1 = f*r^2*v
//   dpx/dk2 = f*r^4*u, dpy/dk2 = f*r^4*v
//
// =============================================================================

void distortWithJacobianCPU(const Eigen::Vector2d& uv, double f, double k1,
                            double k2, Eigen::Vector2d& px_out,
                            Eigen::Matrix2d& dpx_duv,
                            Eigen::Matrix<double, 2, 3>& dpx_dfk1k2) {
  double u = uv[0], v = uv[1];
  double r2 = u * u + v * v;
  double r4 = r2 * r2;
  double D = 1.0 + k1 * r2 + k2 * r4;

  px_out[0] = f * D * u;
  px_out[1] = f * D * v;

  double S = k1 + 2.0 * k2 * r2;

  dpx_duv << f * (2.0 * u * u * S + D), f * 2.0 * u * v * S,
      f * 2.0 * u * v * S, f * (2.0 * v * v * S + D);

  dpx_dfk1k2 << D * u, f * r2 * u, f * r4 * u, D * v, f * r2 * v, f * r4 * v;
}

// =============================================================================
// Full fused projection with Jacobian (CPU)
// =============================================================================
//
// Chain rule through all 4 stages:
//   Stage 1: Rodrigues rotation → P_rot, dProt/domega, dProt/dP
//   Stage 2: Translation → P_cam = P_rot + t
//   Stage 3: Perspective divide → (u,v), d(u,v)/dP_cam
//   Stage 4: Distortion → (px,py), d(px,py)/d(u,v), d(px,py)/d(f,k1,k2)
//
// J_cam (2x9):
//   cols 0-2: C * dProt/domega
//   cols 3-5: C * I_3 = C
//   cols 6-8: d(px,py)/d(f,k1,k2)
//
// J_pt (2x3) = C * R(omega)
//
// where C = d(px,py)/d(u,v) * d(u,v)/dP_cam
//
// =============================================================================

void projectWithJacobianCPU(const double* cam, const double* pt,
                            double* predicted, double* J_cam_out,
                            double* J_pt_out) {
  Eigen::Map<const Eigen::Vector3d> w(cam);
  Eigen::Map<const Eigen::Vector3d> t(cam + 3);
  double f = cam[6], k1 = cam[7], k2 = cam[8];
  Eigen::Map<const Eigen::Vector3d> P_world(pt);

  // Stage 1: Rodrigues rotation
  Eigen::Vector3d P_rot;
  Eigen::Matrix3d dProt_dw, dProt_dP;
  rotatePointWithJacobianCPU(w, P_world, P_rot, dProt_dw, dProt_dP);

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
  // C = d(px,py)/dP_cam = dpx_duv (2x2) . duv_dPcam (2x3) -> 2x3
  Eigen::Matrix<double, 2, 3> C = dpx_duv * duv_dPcam;

  // J_cam (2x9):
  //   cols 0-2: C * dProt_dw
  //   cols 3-5: C * I_3 = C
  //   cols 6-8: dpx_dfk1k2
  Eigen::Matrix<double, 2, 9> J_cam;
  J_cam.block<2, 3>(0, 0) = C * dProt_dw;
  J_cam.block<2, 3>(0, 3) = C;
  J_cam.block<2, 3>(0, 6) = dpx_dfk1k2;

  // J_pt (2x3) = C * R
  Eigen::Matrix<double, 2, 3> J_pt = C * dProt_dP;

  // Copy to output (row-major for consistency with GPU)
  Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J_cam_map(J_cam_out);
  J_cam_map = J_cam;

  Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_pt_map(J_pt_out);
  J_pt_map = J_pt;
}

}  // namespace ops
}  // namespace backend
}  // namespace substral
