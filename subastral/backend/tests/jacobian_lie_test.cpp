#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <random>

#include "subastral/backend/lie/se3.hpp"
#include "subastral/backend/lie/so3.hpp"
#include "subastral/backend/ops/project_with_jacobian_lie_cpu.hpp"
#include "subastral/backend/ops/projection_cpu.hpp"

namespace substral {
namespace backend {
namespace ops {
namespace {

constexpr double kFDStep = 1e-7;

// Relative+absolute tolerance for finite-difference comparison.
// Single-sided FD has O(h) truncation error, so with h=1e-7 we expect
// relative errors around 1e-7 * (second derivative / first derivative).
// For BA projections with distortion, this can be up to ~1e-4 relative.
bool fdNear(double analytical, double fd, double rel_tol = 1e-4,
            double abs_tol = 1e-3) {
  double diff = std::abs(analytical - fd);
  double scale = std::max(std::abs(analytical), std::abs(fd));
  return diff <= abs_tol + rel_tol * scale;
}

// =============================================================================
// Finite-Difference Validation of Lie Group Jacobian
// =============================================================================
//
// The Lie group Jacobian J_cam (2×9) has columns:
//   cols 0-2: ∂(px,py)/∂δφ   (rotation perturbation in se(3))
//   cols 3-5: ∂(px,py)/∂δρ   (translation perturbation in se(3))
//   cols 6-8: ∂(px,py)/∂(δf, δk1, δk2)  (intrinsics, additive)
//
// For the pose part (cols 0-5), the perturbation model is:
//   T_new = Exp(δξ) · T_old
//
// where δξ = (δφ, δρ) ∈ se(3).
//
// To validate column j (j < 6), we:
//   1. Form δξ = h · e_j (h in direction j only)
//   2. Compute T_new = Exp(δξ) · T_old
//   3. Extract (ω_new, t_new) = from T_new
//   4. Project with (ω_new, t_new, f, k1, k2)
//   5. Compare (project_new - project_old) / h with J_cam column j
//
// For the intrinsic part (cols 6-8), we use standard additive perturbation.
//
// =============================================================================

// Helper: project a point using the full BAL pipeline (no Jacobian needed)
void projectOnly(const double* cam, const double* pt, double* predicted) {
  // Use the CPU projection function
  project_cpu(cam, pt, predicted);
}

// Helper: build a 4x4 SE(3) matrix from angle-axis ω and translation t
Eigen::Matrix4d buildSE3(const Eigen::Vector3d& w, const Eigen::Vector3d& t) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = lie::expSO3(w);
  T.block<3, 1>(0, 3) = t;
  return T;
}

TEST(JacobianLie, FiniteDifferencePoseRotation) {
  // Test columns 0-2 (rotation perturbation δφ)
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  for (int trial = 0; trial < 50; ++trial) {
    // Random camera
    double cam[9];
    // ω: random angle-axis with moderate angle
    Eigen::Vector3d w(dist(rng), dist(rng), dist(rng));
    w *= 0.5 + 0.5 * std::abs(dist(rng));  // scale to [0.5, 1.0]
    cam[0] = w(0);
    cam[1] = w(1);
    cam[2] = w(2);
    // t: random translation
    cam[3] = dist(rng) * 5.0;
    cam[4] = dist(rng) * 5.0;
    cam[5] = dist(rng) * 5.0;
    // Intrinsics
    cam[6] = 500.0 + dist(rng) * 200.0;  // f
    cam[7] = dist(rng) * 0.01;           // k1
    cam[8] = dist(rng) * 0.001;          // k2

    // Random 3D point (in front of camera)
    double pt[3] = {dist(rng) * 2.0, dist(rng) * 2.0, 5.0 + dist(rng) * 3.0};

    // Compute analytical Jacobian
    double predicted[2];
    double J_cam[18];  // 2×9
    double J_pt[6];    // 2×3
    projectWithJacobianLieCPU(cam, pt, predicted, J_cam, J_pt);

    // Build current SE(3) element
    Eigen::Vector3d w_cur(cam[0], cam[1], cam[2]);
    Eigen::Vector3d t_cur(cam[3], cam[4], cam[5]);
    Eigen::Matrix4d T_cur = buildSE3(w_cur, t_cur);

    // Finite-difference for columns 0-2 (δφ)
    for (int j = 0; j < 3; ++j) {
      Eigen::Matrix<double, 6, 1> delta_xi =
          Eigen::Matrix<double, 6, 1>::Zero();
      delta_xi(j) = kFDStep;

      // T_new = Exp(δξ) · T_cur
      Eigen::Matrix4d T_new = lie::expSE3(delta_xi) * T_cur;

      // Extract new ω and t
      Eigen::Vector3d w_new = lie::logSO3(T_new.block<3, 3>(0, 0));
      Eigen::Vector3d t_new = T_new.block<3, 1>(0, 3);

      double cam_new[9] = {w_new(0), w_new(1), w_new(2), t_new(0), t_new(1),
                           t_new(2), cam[6],   cam[7],   cam[8]};

      double predicted_new[2];
      projectOnly(cam_new, pt, predicted_new);

      double fd_col0 = (predicted_new[0] - predicted[0]) / kFDStep;
      double fd_col1 = (predicted_new[1] - predicted[1]) / kFDStep;

      // Skip degenerate cases where Jacobian values are enormous
      // (point nearly at camera center, FD breaks down)
      if (std::abs(fd_col0) > 1e10 || std::abs(fd_col1) > 1e10) continue;

      // J_cam is 2×9 row-major:
      //   Row 0: J_cam[0..8]
      //   Row 1: J_cam[9..17]
      EXPECT_TRUE(fdNear(J_cam[j], fd_col0))
          << "Trial " << trial << ", col " << j << " row 0"
          << ": analytical=" << J_cam[j] << " fd=" << fd_col0;
      EXPECT_TRUE(fdNear(J_cam[9 + j], fd_col1))
          << "Trial " << trial << ", col " << j << " row 1"
          << ": analytical=" << J_cam[9 + j] << " fd=" << fd_col1;
    }
  }
}

TEST(JacobianLie, FiniteDifferencePoseTranslation) {
  // Test columns 3-5 (translation perturbation δρ)
  std::mt19937 rng(123);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  for (int trial = 0; trial < 50; ++trial) {
    double cam[9];
    Eigen::Vector3d w(dist(rng), dist(rng), dist(rng));
    w *= 0.5 + 0.5 * std::abs(dist(rng));
    cam[0] = w(0);
    cam[1] = w(1);
    cam[2] = w(2);
    cam[3] = dist(rng) * 5.0;
    cam[4] = dist(rng) * 5.0;
    cam[5] = dist(rng) * 5.0;
    cam[6] = 500.0 + dist(rng) * 200.0;
    cam[7] = dist(rng) * 0.01;
    cam[8] = dist(rng) * 0.001;

    double pt[3] = {dist(rng) * 2.0, dist(rng) * 2.0, 5.0 + dist(rng) * 3.0};

    double predicted[2];
    double J_cam[18];
    double J_pt[6];
    projectWithJacobianLieCPU(cam, pt, predicted, J_cam, J_pt);

    Eigen::Vector3d w_cur(cam[0], cam[1], cam[2]);
    Eigen::Vector3d t_cur(cam[3], cam[4], cam[5]);
    Eigen::Matrix4d T_cur = buildSE3(w_cur, t_cur);

    // Finite-difference for columns 3-5 (δρ)
    for (int j = 0; j < 3; ++j) {
      Eigen::Matrix<double, 6, 1> delta_xi =
          Eigen::Matrix<double, 6, 1>::Zero();
      delta_xi(3 + j) = kFDStep;  // translation perturbation

      Eigen::Matrix4d T_new = lie::expSE3(delta_xi) * T_cur;

      Eigen::Vector3d w_new = lie::logSO3(T_new.block<3, 3>(0, 0));
      Eigen::Vector3d t_new = T_new.block<3, 1>(0, 3);

      double cam_new[9] = {w_new(0), w_new(1), w_new(2), t_new(0), t_new(1),
                           t_new(2), cam[6],   cam[7],   cam[8]};

      double predicted_new[2];
      projectOnly(cam_new, pt, predicted_new);

      double fd_col0 = (predicted_new[0] - predicted[0]) / kFDStep;
      double fd_col1 = (predicted_new[1] - predicted[1]) / kFDStep;

      int col = 3 + j;
      EXPECT_TRUE(fdNear(J_cam[col], fd_col0))
          << "Trial " << trial << ", col " << col << " row 0"
          << ": analytical=" << J_cam[col] << " fd=" << fd_col0;
      EXPECT_TRUE(fdNear(J_cam[9 + col], fd_col1))
          << "Trial " << trial << ", col " << col << " row 1"
          << ": analytical=" << J_cam[9 + col] << " fd=" << fd_col1;
    }
  }
}

TEST(JacobianLie, FiniteDifferenceIntrinsics) {
  // Test columns 6-8 (f, k1, k2 — standard additive)
  std::mt19937 rng(456);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  for (int trial = 0; trial < 50; ++trial) {
    double cam[9];
    Eigen::Vector3d w(dist(rng), dist(rng), dist(rng));
    w *= 0.5 + 0.5 * std::abs(dist(rng));
    cam[0] = w(0);
    cam[1] = w(1);
    cam[2] = w(2);
    cam[3] = dist(rng) * 5.0;
    cam[4] = dist(rng) * 5.0;
    cam[5] = dist(rng) * 5.0;
    cam[6] = 500.0 + dist(rng) * 200.0;
    cam[7] = dist(rng) * 0.01;
    cam[8] = dist(rng) * 0.001;

    double pt[3] = {dist(rng) * 2.0, dist(rng) * 2.0, 5.0 + dist(rng) * 3.0};

    double predicted[2];
    double J_cam[18];
    double J_pt[6];
    projectWithJacobianLieCPU(cam, pt, predicted, J_cam, J_pt);

    // Finite-difference for columns 6-8 (f, k1, k2)
    for (int j = 0; j < 3; ++j) {
      double cam_new[9];
      for (int k = 0; k < 9; ++k) cam_new[k] = cam[k];
      cam_new[6 + j] += kFDStep;

      double predicted_new[2];
      projectOnly(cam_new, pt, predicted_new);

      double fd_col0 = (predicted_new[0] - predicted[0]) / kFDStep;
      double fd_col1 = (predicted_new[1] - predicted[1]) / kFDStep;

      int col = 6 + j;
      EXPECT_TRUE(fdNear(J_cam[col], fd_col0))
          << "Trial " << trial << ", col " << col << " row 0"
          << ": analytical=" << J_cam[col] << " fd=" << fd_col0;
      EXPECT_TRUE(fdNear(J_cam[9 + col], fd_col1))
          << "Trial " << trial << ", col " << col << " row 1"
          << ": analytical=" << J_cam[9 + col] << " fd=" << fd_col1;
    }
  }
}

TEST(JacobianLie, FiniteDifferencePoint) {
  // Test J_pt (2×3) — standard additive perturbation on point
  std::mt19937 rng(789);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  for (int trial = 0; trial < 50; ++trial) {
    double cam[9];
    Eigen::Vector3d w(dist(rng), dist(rng), dist(rng));
    w *= 0.5 + 0.5 * std::abs(dist(rng));
    cam[0] = w(0);
    cam[1] = w(1);
    cam[2] = w(2);
    cam[3] = dist(rng) * 5.0;
    cam[4] = dist(rng) * 5.0;
    cam[5] = dist(rng) * 5.0;
    cam[6] = 500.0 + dist(rng) * 200.0;
    cam[7] = dist(rng) * 0.01;
    cam[8] = dist(rng) * 0.001;

    double pt[3] = {dist(rng) * 2.0, dist(rng) * 2.0, 5.0 + dist(rng) * 3.0};

    double predicted[2];
    double J_cam[18];
    double J_pt[6];  // 2×3 row-major
    projectWithJacobianLieCPU(cam, pt, predicted, J_cam, J_pt);

    for (int j = 0; j < 3; ++j) {
      double pt_new[3] = {pt[0], pt[1], pt[2]};
      pt_new[j] += kFDStep;

      double predicted_new[2];
      projectOnly(cam, pt_new, predicted_new);

      double fd_col0 = (predicted_new[0] - predicted[0]) / kFDStep;
      double fd_col1 = (predicted_new[1] - predicted[1]) / kFDStep;

      EXPECT_TRUE(fdNear(J_pt[j], fd_col0))
          << "Trial " << trial << ", col " << j << " row 0"
          << ": analytical=" << J_pt[j] << " fd=" << fd_col0;
      EXPECT_TRUE(fdNear(J_pt[3 + j], fd_col1))
          << "Trial " << trial << ", col " << j << " row 1"
          << ": analytical=" << J_pt[3 + j] << " fd=" << fd_col1;
    }
  }
}

}  // namespace
}  // namespace ops
}  // namespace backend
}  // namespace substral
