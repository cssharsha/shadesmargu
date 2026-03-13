#include "subastral/backend/lie/so3.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <random>

namespace substral {
namespace backend {
namespace lie {
namespace {

// =============================================================================
// Test Utilities
// =============================================================================

constexpr double kTol = 1e-10;
constexpr double kLooseTol = 1e-8;

Eigen::Vector3d randomAngleAxis(std::mt19937& rng, double max_angle) {
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  Eigen::Vector3d w(dist(rng), dist(rng), dist(rng));
  if (w.norm() < 1e-12) w = Eigen::Vector3d(1, 0, 0);
  w.normalize();
  std::uniform_real_distribution<double> angle_dist(0.01, max_angle);
  return w * angle_dist(rng);
}

// =============================================================================
// hat / vee
// =============================================================================

TEST(SO3, HatVeeRoundtrip) {
  Eigen::Vector3d w(0.3, -0.7, 1.2);
  Eigen::Matrix3d W = hat(w);

  // Verify skew-symmetric
  EXPECT_NEAR((W + W.transpose()).norm(), 0.0, kTol);

  // Verify vee recovers the vector
  Eigen::Vector3d w_recovered = vee(W);
  EXPECT_NEAR((w - w_recovered).norm(), 0.0, kTol);
}

TEST(SO3, HatCrossProduct) {
  // hat(w) * v should equal w × v
  Eigen::Vector3d w(1.0, 2.0, 3.0);
  Eigen::Vector3d v(-0.5, 0.8, 1.3);

  Eigen::Vector3d cross_hat = hat(w) * v;
  Eigen::Vector3d cross_direct = w.cross(v);

  EXPECT_NEAR((cross_hat - cross_direct).norm(), 0.0, kTol);
}

// =============================================================================
// exp: basic properties
// =============================================================================

TEST(SO3, ExpZeroIsIdentity) {
  Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R = expSO3(zero);
  EXPECT_NEAR((R - Eigen::Matrix3d::Identity()).norm(), 0.0, kTol);
}

TEST(SO3, ExpProducesValidRotation) {
  // Test several random angle-axis vectors
  std::mt19937 rng(42);
  for (int i = 0; i < 100; ++i) {
    Eigen::Vector3d w = randomAngleAxis(rng, M_PI);
    Eigen::Matrix3d R = expSO3(w);

    // R^T R = I
    EXPECT_NEAR((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 0.0,
                kTol);
    // det R = +1
    EXPECT_NEAR(R.determinant(), 1.0, kTol);
  }
}

TEST(SO3, ExpSmallAngle) {
  // Very small angle — should still produce valid rotation
  Eigen::Vector3d w(1e-12, 2e-12, -3e-12);
  Eigen::Matrix3d R = expSO3(w);

  EXPECT_NEAR((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 0.0,
              kTol);
  EXPECT_NEAR(R.determinant(), 1.0, kTol);
}

TEST(SO3, ExpPiRotation) {
  // 180-degree rotation about x-axis
  Eigen::Vector3d w(M_PI, 0, 0);
  Eigen::Matrix3d R = expSO3(w);

  // R_x(π) = diag(1, -1, -1)
  Eigen::Matrix3d expected;
  expected << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  EXPECT_NEAR((R - expected).norm(), 0.0, kLooseTol);
}

TEST(SO3, ExpHalfPiRotation) {
  // 90-degree rotation about z-axis
  Eigen::Vector3d w(0, 0, M_PI / 2.0);
  Eigen::Matrix3d R = expSO3(w);

  // R_z(π/2) rotates x→y, y→-x
  Eigen::Matrix3d expected;
  expected << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  EXPECT_NEAR((R - expected).norm(), 0.0, kLooseTol);
}

// =============================================================================
// log: basic properties
// =============================================================================

TEST(SO3, LogIdentityIsZero) {
  Eigen::Vector3d w = logSO3(Eigen::Matrix3d::Identity());
  EXPECT_NEAR(w.norm(), 0.0, kTol);
}

TEST(SO3, LogExpRoundtrip) {
  // exp(log(R)) = R for random rotations
  std::mt19937 rng(123);
  for (int i = 0; i < 100; ++i) {
    Eigen::Vector3d w_orig = randomAngleAxis(rng, M_PI - 0.01);
    Eigen::Matrix3d R = expSO3(w_orig);
    Eigen::Vector3d w_recovered = logSO3(R);
    Eigen::Matrix3d R_recovered = expSO3(w_recovered);

    EXPECT_NEAR((R - R_recovered).norm(), 0.0, kLooseTol)
        << "Failed for w = " << w_orig.transpose()
        << " (theta = " << w_orig.norm() << ")";
  }
}

TEST(SO3, ExpLogRoundtrip) {
  // log(exp(w)) = w for angles < π
  std::mt19937 rng(456);
  for (int i = 0; i < 100; ++i) {
    Eigen::Vector3d w_orig = randomAngleAxis(rng, M_PI - 0.01);
    Eigen::Matrix3d R = expSO3(w_orig);
    Eigen::Vector3d w_recovered = logSO3(R);

    // The angle-axis should be the same (for θ < π, the representation is
    // unique)
    EXPECT_NEAR((w_orig - w_recovered).norm(), 0.0, kLooseTol)
        << "Failed for w = " << w_orig.transpose();
  }
}

TEST(SO3, LogNearPi) {
  // Test log near θ = π (the tricky case)
  Eigen::Vector3d w(M_PI - 0.001, 0, 0);
  Eigen::Matrix3d R = expSO3(w);
  Eigen::Vector3d w_recovered = logSO3(R);
  Eigen::Matrix3d R_recovered = expSO3(w_recovered);

  EXPECT_NEAR((R - R_recovered).norm(), 0.0, kLooseTol);
}

// =============================================================================
// Left Jacobian: properties
// =============================================================================

TEST(SO3, LeftJacobianIdentityAtZero) {
  // J_l(0) = I
  Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  Eigen::Matrix3d Jl = leftJacobianSO3(zero);
  EXPECT_NEAR((Jl - Eigen::Matrix3d::Identity()).norm(), 0.0, kTol);
}

TEST(SO3, LeftJacobianInverseProduct) {
  // J_l(w) * J_l^{-1}(w) = I
  std::mt19937 rng(789);
  for (int i = 0; i < 50; ++i) {
    Eigen::Vector3d w = randomAngleAxis(rng, M_PI - 0.01);
    Eigen::Matrix3d Jl = leftJacobianSO3(w);
    Eigen::Matrix3d Jl_inv = leftJacobianInverseSO3(w);

    EXPECT_NEAR((Jl * Jl_inv - Eigen::Matrix3d::Identity()).norm(), 0.0,
                kLooseTol)
        << "Failed for w = " << w.transpose();
  }
}

TEST(SO3, LeftJacobianNumericalVerification) {
  // Verify J_l by finite differences:
  //   exp(w + eps*e_j) ≈ exp(J_l(w) * eps*e_j) * exp(w)
  //
  // Equivalently, check that:
  //   d/deps log(exp(w + eps*e_j) * exp(-w)) |_{eps=0} ≈ J_l(w) * e_j
  //
  // We use the simpler approach:
  //   (exp(w + h*e_j) - exp(w)) / h  ≈ d(exp(w))/dw_j
  //
  // And the relationship: d(exp(w))/dw = ... involves J_l.

  double h = 1e-7;
  std::mt19937 rng(321);

  for (int trial = 0; trial < 20; ++trial) {
    Eigen::Vector3d w = randomAngleAxis(rng, 2.0);
    Eigen::Matrix3d Jl = leftJacobianSO3(w);
    Eigen::Matrix3d R = expSO3(w);

    for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d w_plus = w;
      w_plus(j) += h;

      Eigen::Matrix3d R_plus = expSO3(w_plus);

      // log(R_plus * R^{-1}) should approximate J_l * (h * e_j)
      Eigen::Matrix3d dR = R_plus * R.transpose();
      Eigen::Vector3d delta = logSO3(dR);

      Eigen::Vector3d ej = Eigen::Vector3d::Zero();
      ej(j) = 1.0;
      Eigen::Vector3d expected = Jl * (h * ej);

      EXPECT_NEAR((delta - expected).norm(), 0.0, 1e-5)
          << "Trial " << trial << ", j=" << j << "\n  w = " << w.transpose()
          << "\n  delta = " << delta.transpose()
          << "\n  expected = " << expected.transpose();
    }
  }
}

TEST(SO3, RightJacobianRelation) {
  // J_r(w) = J_l(-w)
  // Also: J_r(w) = R^T * J_l(w)
  std::mt19937 rng(654);
  for (int i = 0; i < 50; ++i) {
    Eigen::Vector3d w = randomAngleAxis(rng, M_PI - 0.01);
    Eigen::Matrix3d Jr = rightJacobianSO3(w);
    Eigen::Matrix3d Jl = leftJacobianSO3(w);
    Eigen::Matrix3d R = expSO3(w);

    // J_r = R^T * J_l
    Eigen::Matrix3d Jr_from_Jl = R.transpose() * Jl;
    EXPECT_NEAR((Jr - Jr_from_Jl).norm(), 0.0, kLooseTol);
  }
}

// =============================================================================
// Consistency with existing Rodrigues implementation
// =============================================================================

TEST(SO3, ExpMatchesRodriguesFormula) {
  // Verify that our expSO3 produces the same rotation matrix as the
  // Rodrigues formula used in rodrigues_gpu.cuh / projection_cpu.cpp
  std::mt19937 rng(111);
  for (int i = 0; i < 50; ++i) {
    Eigen::Vector3d w = randomAngleAxis(rng, M_PI);
    double theta = w.norm();
    Eigen::Vector3d k = w / theta;

    // Manual Rodrigues: R = I*cosθ + (1-cosθ)*k*k^T + sinθ*[k]×
    double ct = std::cos(theta);
    double st = std::sin(theta);
    Eigen::Matrix3d R_rodrigues = ct * Eigen::Matrix3d::Identity() +
                                  (1.0 - ct) * (k * k.transpose()) +
                                  st * hat(k);

    Eigen::Matrix3d R_exp = expSO3(w);

    EXPECT_NEAR((R_rodrigues - R_exp).norm(), 0.0, kTol);
  }
}

}  // namespace
}  // namespace lie
}  // namespace backend
}  // namespace substral
