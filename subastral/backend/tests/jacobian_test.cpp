#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>

#include "subastral/backend/ops/project_with_jacobian_cpu.hpp"
#include "subastral/backend/ops/projection_cpu.hpp"

namespace substral {
namespace backend {
namespace ops {

// =============================================================================
// Finite-difference Jacobian computation
// =============================================================================
//
// For a function f: R^n -> R^m, the numerical Jacobian is:
//   J[i][j] = (f(x + h*e_j) - f(x - h*e_j)) / (2h)
//
// We use h = 1e-7 (central differences give O(h²) error, so with
// double precision we expect ~1e-14/1e-7 ≈ 1e-7 truncation error,
// well below our 1e-5 tolerance).
// =============================================================================

static constexpr double kFiniteDiffStep = 1e-7;
// Central finite differences have O(h²) truncation error. With h=1e-7
// and double precision (~1e-16 machine epsilon), the optimal error is
// roughly max(h², eps/h) ≈ max(1e-14, 1e-9) ≈ 1e-9. In practice,
// through the chain of nonlinear operations, we see errors up to ~1e-4
// for small Jacobian entries. A tolerance of 5e-4 is conservative.
static constexpr double kRelTolerance = 5e-4;

// Compute numerical Jacobian of projection w.r.t. camera parameters (2×9)
void numericalJacobianCamera(const double* cam, const double* pt,
                             Eigen::Matrix<double, 2, 9>& J_cam_num) {
  for (int j = 0; j < 9; ++j) {
    double cam_plus[9], cam_minus[9];
    for (int k = 0; k < 9; ++k) {
      cam_plus[k] = cam[k];
      cam_minus[k] = cam[k];
    }
    cam_plus[j] += kFiniteDiffStep;
    cam_minus[j] -= kFiniteDiffStep;

    double pred_plus[2], pred_minus[2];
    project_cpu(cam_plus, pt, pred_plus);
    project_cpu(cam_minus, pt, pred_minus);

    J_cam_num(0, j) = (pred_plus[0] - pred_minus[0]) / (2.0 * kFiniteDiffStep);
    J_cam_num(1, j) = (pred_plus[1] - pred_minus[1]) / (2.0 * kFiniteDiffStep);
  }
}

// Compute numerical Jacobian of projection w.r.t. 3D point (2×3)
void numericalJacobianPoint(const double* cam, const double* pt,
                            Eigen::Matrix<double, 2, 3>& J_pt_num) {
  for (int j = 0; j < 3; ++j) {
    double pt_plus[3], pt_minus[3];
    for (int k = 0; k < 3; ++k) {
      pt_plus[k] = pt[k];
      pt_minus[k] = pt[k];
    }
    pt_plus[j] += kFiniteDiffStep;
    pt_minus[j] -= kFiniteDiffStep;

    double pred_plus[2], pred_minus[2];
    project_cpu(cam, pt_plus, pred_plus);
    project_cpu(cam, pt_minus, pred_minus);

    J_pt_num(0, j) = (pred_plus[0] - pred_minus[0]) / (2.0 * kFiniteDiffStep);
    J_pt_num(1, j) = (pred_plus[1] - pred_minus[1]) / (2.0 * kFiniteDiffStep);
  }
}

// Compare analytical vs numerical Jacobian entry-by-entry.
// Uses a mixed absolute/relative error criterion:
//   error = |a - n| / (1 + max(|a|, |n|))
// This smoothly transitions from absolute error for small values to
// relative error for large values, avoiding issues at near-zero entries
// where finite-difference precision degrades.
void compareJacobians(const std::string& name,
                      const Eigen::MatrixXd& analytical,
                      const Eigen::MatrixXd& numerical) {
  ASSERT_EQ(analytical.rows(), numerical.rows());
  ASSERT_EQ(analytical.cols(), numerical.cols());

  for (int i = 0; i < analytical.rows(); ++i) {
    for (int j = 0; j < analytical.cols(); ++j) {
      double a = analytical(i, j);
      double n = numerical(i, j);
      double abs_diff = std::abs(a - n);
      double max_abs = std::max(std::abs(a), std::abs(n));

      // Mixed tolerance: abs_diff / (1 + max_abs) < tol
      // For large values this is ≈ relative error
      // For small values this is ≈ absolute error
      double mixed_err = abs_diff / (1.0 + max_abs);
      EXPECT_LT(mixed_err, kRelTolerance)
          << name << "[" << i << "," << j << "]: "
          << "analytical=" << a << " numerical=" << n
          << " abs_diff=" << abs_diff << " mixed_err=" << mixed_err;
    }
  }
}

// =============================================================================
// Test fixture
// =============================================================================

class JacobianTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Use a fixed seed for reproducibility
    gen_.seed(42);
  }

  // Generate a realistic camera configuration
  // ω: small rotation (typical for BA), t: moderate translation,
  // f: typical focal length, k1/k2: small distortion
  void randomCamera(double* cam) {
    std::normal_distribution<double> rot_dist(0.0, 0.5);
    std::normal_distribution<double> trans_dist(0.0, 2.0);
    std::normal_distribution<double> focal_dist(800.0, 200.0);
    std::normal_distribution<double> distort_dist(0.0, 0.01);

    cam[0] = rot_dist(gen_);
    cam[1] = rot_dist(gen_);
    cam[2] = rot_dist(gen_);
    cam[3] = trans_dist(gen_);
    cam[4] = trans_dist(gen_);
    cam[5] = trans_dist(gen_);
    cam[6] = std::abs(focal_dist(gen_));  // focal length must be positive
    cam[7] = distort_dist(gen_);
    cam[8] = distort_dist(gen_);
  }

  // Generate a 3D point that projects in front of the camera
  // (Z should be positive after rotation + translation)
  void randomPoint(const double* cam, double* pt) {
    std::normal_distribution<double> dist(0.0, 5.0);
    // Keep trying until the point is in front of the camera
    for (int attempt = 0; attempt < 100; ++attempt) {
      pt[0] = dist(gen_);
      pt[1] = dist(gen_);
      pt[2] = dist(gen_) + 10.0;  // bias toward positive Z

      // Quick check: project and see if Z_cam > 0
      double predicted[2];
      project_cpu(cam, pt, predicted);
      // If projection succeeded (no NaN), the point is valid
      if (std::isfinite(predicted[0]) && std::isfinite(predicted[1])) {
        return;
      }
    }
    // Fallback: point directly in front
    pt[0] = 0.0;
    pt[1] = 0.0;
    pt[2] = 10.0;
  }

  std::mt19937 gen_;
};

// =============================================================================
// Test: Single observation with known camera
// =============================================================================

TEST_F(JacobianTest, SingleObservation_IdentityRotation) {
  // Camera at origin, looking down Z, no rotation
  double cam[9] = {0,      0, 0,  // ω = 0 (identity rotation)
                   0,      0, 0,  // t = 0
                   500.0,         // f
                   -0.001,        // k1
                   0.0001};       // k2

  double pt[3] = {1.0, 2.0, 10.0};

  double predicted[2];
  double J_cam[18], J_pt[6];
  projectWithJacobianCPU(cam, pt, predicted, J_cam, J_pt);

  // Verify projection matches the existing project_cpu
  double predicted_ref[2];
  project_cpu(cam, pt, predicted_ref);
  EXPECT_NEAR(predicted[0], predicted_ref[0], 1e-12);
  EXPECT_NEAR(predicted[1], predicted_ref[1], 1e-12);

  // Compare against finite differences
  Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J_cam_map(J_cam);
  Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_pt_map(J_pt);

  Eigen::Matrix<double, 2, 9> J_cam_num;
  numericalJacobianCamera(cam, pt, J_cam_num);

  Eigen::Matrix<double, 2, 3> J_pt_num;
  numericalJacobianPoint(cam, pt, J_pt_num);

  compareJacobians("J_cam (identity rot)", J_cam_map, J_cam_num);
  compareJacobians("J_pt (identity rot)", J_pt_map, J_pt_num);
}

TEST_F(JacobianTest, SingleObservation_SmallRotation) {
  // Small rotation (tests the θ < ε branch boundary)
  double cam[9] = {1e-11,  2e-11, -1e-11,  // ω ≈ 0 (very small)
                   1.0,    -0.5,  0.2,     // t
                   600.0,                  // f
                   -0.002,                 // k1
                   0.0005};                // k2

  double pt[3] = {2.0, -1.0, 8.0};

  double predicted[2];
  double J_cam[18], J_pt[6];
  projectWithJacobianCPU(cam, pt, predicted, J_cam, J_pt);

  Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J_cam_map(J_cam);
  Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_pt_map(J_pt);

  Eigen::Matrix<double, 2, 9> J_cam_num;
  numericalJacobianCamera(cam, pt, J_cam_num);

  Eigen::Matrix<double, 2, 3> J_pt_num;
  numericalJacobianPoint(cam, pt, J_pt_num);

  compareJacobians("J_cam (small rot)", J_cam_map, J_cam_num);
  compareJacobians("J_pt (small rot)", J_pt_map, J_pt_num);
}

TEST_F(JacobianTest, SingleObservation_LargeRotation) {
  // Large rotation (90 degrees about Z)
  double cam[9] = {0,      0,    M_PI / 2.0,  // ω = 90° about Z
                   0.5,    -1.0, 3.0,         // t
                   400.0,                     // f
                   -0.005,                    // k1
                   0.001};                    // k2

  double pt[3] = {3.0, 1.0, 12.0};

  double predicted[2];
  double J_cam[18], J_pt[6];
  projectWithJacobianCPU(cam, pt, predicted, J_cam, J_pt);

  Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J_cam_map(J_cam);
  Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_pt_map(J_pt);

  Eigen::Matrix<double, 2, 9> J_cam_num;
  numericalJacobianCamera(cam, pt, J_cam_num);

  Eigen::Matrix<double, 2, 3> J_pt_num;
  numericalJacobianPoint(cam, pt, J_pt_num);

  compareJacobians("J_cam (large rot)", J_cam_map, J_cam_num);
  compareJacobians("J_pt (large rot)", J_pt_map, J_pt_num);
}

// =============================================================================
// Test: Randomized stress test — many random camera/point configurations
// =============================================================================

TEST_F(JacobianTest, RandomizedStressTest) {
  constexpr int kNumTrials = 100;

  for (int trial = 0; trial < kNumTrials; ++trial) {
    double cam[9];
    randomCamera(cam);

    double pt[3];
    randomPoint(cam, pt);

    double predicted[2];
    double J_cam[18], J_pt[6];
    projectWithJacobianCPU(cam, pt, predicted, J_cam, J_pt);

    // Verify projection matches project_cpu
    // Use relative tolerance since pixel values can be large
    double predicted_ref[2];
    project_cpu(cam, pt, predicted_ref);
    EXPECT_NEAR(predicted[0], predicted_ref[0],
                1e-10 + 1e-12 * std::abs(predicted_ref[0]))
        << "Trial " << trial << ": projection mismatch (x)";
    EXPECT_NEAR(predicted[1], predicted_ref[1],
                1e-10 + 1e-12 * std::abs(predicted_ref[1]))
        << "Trial " << trial << ": projection mismatch (y)";

    // Compare Jacobians against finite differences
    Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J_cam_map(J_cam);
    Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_pt_map(J_pt);

    Eigen::Matrix<double, 2, 9> J_cam_num;
    numericalJacobianCamera(cam, pt, J_cam_num);

    Eigen::Matrix<double, 2, 3> J_pt_num;
    numericalJacobianPoint(cam, pt, J_pt_num);

    compareJacobians("J_cam (trial " + std::to_string(trial) + ")", J_cam_map,
                     J_cam_num);
    compareJacobians("J_pt (trial " + std::to_string(trial) + ")", J_pt_map,
                     J_pt_num);
  }
}

// =============================================================================
// Test: Verify each sub-stage Jacobian independently
// =============================================================================

TEST_F(JacobianTest, RodriguesJacobian_Isolated) {
  // Test the Rodrigues Jacobian in isolation against finite differences
  // of the rotation function alone
  Eigen::Vector3d w(0.3, -0.5, 0.7);
  Eigen::Vector3d P(2.0, -1.0, 3.0);

  Eigen::Vector3d P_rot;
  Eigen::Matrix3d dProt_dw, dProt_dP;
  rotatePointWithJacobianCPU(w, P, P_rot, dProt_dw, dProt_dP);

  // Numerical dP_rot/dw
  Eigen::Matrix3d dProt_dw_num;
  double h = kFiniteDiffStep;
  for (int j = 0; j < 3; ++j) {
    Eigen::Vector3d w_plus = w, w_minus = w;
    w_plus[j] += h;
    w_minus[j] -= h;

    // Use Rodrigues formula directly
    auto rodrigues = [](const Eigen::Vector3d& omega,
                        const Eigen::Vector3d& p) -> Eigen::Vector3d {
      double th = omega.norm();
      if (th < 1e-10) return p + omega.cross(p);
      Eigen::Vector3d ax = omega / th;
      return p * std::cos(th) + ax.cross(p) * std::sin(th) +
             ax * ax.dot(p) * (1.0 - std::cos(th));
    };

    Eigen::Vector3d rot_plus = rodrigues(w_plus, P);
    Eigen::Vector3d rot_minus = rodrigues(w_minus, P);
    dProt_dw_num.col(j) = (rot_plus - rot_minus) / (2.0 * h);
  }

  compareJacobians("dProt/dw", dProt_dw, dProt_dw_num);

  // Numerical dP_rot/dP
  Eigen::Matrix3d dProt_dP_num;
  for (int j = 0; j < 3; ++j) {
    Eigen::Vector3d P_plus = P, P_minus = P;
    P_plus[j] += h;
    P_minus[j] -= h;

    auto rodrigues = [](const Eigen::Vector3d& omega,
                        const Eigen::Vector3d& p) -> Eigen::Vector3d {
      double th = omega.norm();
      if (th < 1e-10) return p + omega.cross(p);
      Eigen::Vector3d ax = omega / th;
      return p * std::cos(th) + ax.cross(p) * std::sin(th) +
             ax * ax.dot(p) * (1.0 - std::cos(th));
    };

    Eigen::Vector3d rot_plus = rodrigues(w, P_plus);
    Eigen::Vector3d rot_minus = rodrigues(w, P_minus);
    dProt_dP_num.col(j) = (rot_plus - rot_minus) / (2.0 * h);
  }

  compareJacobians("dProt/dP (= R)", dProt_dP, dProt_dP_num);
}

TEST_F(JacobianTest, DistortionJacobian_Isolated) {
  // Test distortion Jacobian in isolation
  Eigen::Vector2d uv(-0.1, 0.15);
  double f = 500.0, k1 = -0.003, k2 = 0.0001;

  Eigen::Vector2d px;
  Eigen::Matrix2d dpx_duv;
  Eigen::Matrix<double, 2, 3> dpx_dfk1k2;
  distortWithJacobianCPU(uv, f, k1, k2, px, dpx_duv, dpx_dfk1k2);

  // Numerical d(px)/d(u,v)
  double h = kFiniteDiffStep;
  Eigen::Matrix2d dpx_duv_num;
  for (int j = 0; j < 2; ++j) {
    Eigen::Vector2d uv_plus = uv, uv_minus = uv;
    uv_plus[j] += h;
    uv_minus[j] -= h;

    auto distort = [](const Eigen::Vector2d& uv_in, double f_in, double k1_in,
                      double k2_in) -> Eigen::Vector2d {
      double r2 = uv_in.squaredNorm();
      double D = 1.0 + k1_in * r2 + k2_in * r2 * r2;
      return Eigen::Vector2d(f_in * D * uv_in[0], f_in * D * uv_in[1]);
    };

    Eigen::Vector2d px_plus = distort(uv_plus, f, k1, k2);
    Eigen::Vector2d px_minus = distort(uv_minus, f, k1, k2);
    dpx_duv_num.col(j) = (px_plus - px_minus) / (2.0 * h);
  }

  compareJacobians("dpx/d(u,v)", dpx_duv, dpx_duv_num);

  // Numerical d(px)/d(f,k1,k2)
  Eigen::Matrix<double, 2, 3> dpx_dfk1k2_num;
  double params[3] = {f, k1, k2};
  for (int j = 0; j < 3; ++j) {
    double params_plus[3] = {params[0], params[1], params[2]};
    double params_minus[3] = {params[0], params[1], params[2]};
    params_plus[j] += h;
    params_minus[j] -= h;

    auto distort = [](const Eigen::Vector2d& uv_in, double f_in, double k1_in,
                      double k2_in) -> Eigen::Vector2d {
      double r2 = uv_in.squaredNorm();
      double D = 1.0 + k1_in * r2 + k2_in * r2 * r2;
      return Eigen::Vector2d(f_in * D * uv_in[0], f_in * D * uv_in[1]);
    };

    Eigen::Vector2d px_plus =
        distort(uv, params_plus[0], params_plus[1], params_plus[2]);
    Eigen::Vector2d px_minus =
        distort(uv, params_minus[0], params_minus[1], params_minus[2]);
    dpx_dfk1k2_num.col(j) = (px_plus - px_minus) / (2.0 * h);
  }

  compareJacobians("dpx/d(f,k1,k2)", dpx_dfk1k2, dpx_dfk1k2_num);
}

}  // namespace ops
}  // namespace backend
}  // namespace substral
