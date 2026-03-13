#include "subastral/backend/lie/se3.hpp"

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

Eigen::Matrix<double, 6, 1> randomTwist(std::mt19937& rng, double max_angle,
                                        double max_trans) {
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  Eigen::Vector3d w(dist(rng), dist(rng), dist(rng));
  if (w.norm() < 1e-12) w = Eigen::Vector3d(1, 0, 0);
  w.normalize();
  std::uniform_real_distribution<double> angle_dist(0.01, max_angle);
  w *= angle_dist(rng);

  Eigen::Vector3d v(dist(rng) * max_trans, dist(rng) * max_trans,
                    dist(rng) * max_trans);

  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = w;
  xi.tail<3>() = v;
  return xi;
}

Eigen::Matrix4d randomSE3(std::mt19937& rng) {
  Eigen::Matrix<double, 6, 1> xi = randomTwist(rng, M_PI - 0.01, 5.0);
  return expSE3(xi);
}

bool isValidSE3(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  // Check R^T R = I
  if ((R.transpose() * R - Eigen::Matrix3d::Identity()).norm() > 1e-8)
    return false;
  // Check det R = +1
  if (std::abs(R.determinant() - 1.0) > 1e-8) return false;
  // Check bottom row
  if (std::abs(T(3, 0)) > 1e-12 || std::abs(T(3, 1)) > 1e-12 ||
      std::abs(T(3, 2)) > 1e-12 || std::abs(T(3, 3) - 1.0) > 1e-12)
    return false;
  return true;
}

// =============================================================================
// hatSE3 / veeSE3
// =============================================================================

TEST(SE3, HatVeeRoundtrip) {
  Eigen::Matrix<double, 6, 1> xi;
  xi << 0.1, -0.2, 0.3, 1.0, -2.0, 3.0;

  Eigen::Matrix4d X = hatSE3(xi);
  Eigen::Matrix<double, 6, 1> xi_recovered = veeSE3(X);

  EXPECT_NEAR((xi - xi_recovered).norm(), 0.0, kTol);
}

TEST(SE3, HatStructure) {
  Eigen::Matrix<double, 6, 1> xi;
  xi << 0.1, -0.2, 0.3, 4.0, 5.0, 6.0;

  Eigen::Matrix4d X = hatSE3(xi);

  // Top-left 3×3 should be skew-symmetric
  Eigen::Matrix3d W = X.block<3, 3>(0, 0);
  EXPECT_NEAR((W + W.transpose()).norm(), 0.0, kTol);

  // Top-right should be the translation part
  EXPECT_NEAR(X(0, 3), 4.0, kTol);
  EXPECT_NEAR(X(1, 3), 5.0, kTol);
  EXPECT_NEAR(X(2, 3), 6.0, kTol);

  // Bottom row should be zero
  EXPECT_NEAR(X(3, 0), 0.0, kTol);
  EXPECT_NEAR(X(3, 1), 0.0, kTol);
  EXPECT_NEAR(X(3, 2), 0.0, kTol);
  EXPECT_NEAR(X(3, 3), 0.0, kTol);
}

// =============================================================================
// exp: basic properties
// =============================================================================

TEST(SE3, ExpZeroIsIdentity) {
  Eigen::Matrix<double, 6, 1> zero = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix4d T = expSE3(zero);
  EXPECT_NEAR((T - Eigen::Matrix4d::Identity()).norm(), 0.0, kTol);
}

TEST(SE3, ExpProducesValidSE3) {
  std::mt19937 rng(42);
  for (int i = 0; i < 100; ++i) {
    Eigen::Matrix<double, 6, 1> xi = randomTwist(rng, M_PI - 0.01, 5.0);
    Eigen::Matrix4d T = expSE3(xi);
    EXPECT_TRUE(isValidSE3(T)) << "Failed for xi = " << xi.transpose();
  }
}

TEST(SE3, ExpPureRotation) {
  // When v = 0, exp should give a pure rotation (no translation)
  Eigen::Matrix<double, 6, 1> xi;
  xi << 0.0, 0.0, M_PI / 2.0, 0.0, 0.0, 0.0;

  Eigen::Matrix4d T = expSE3(xi);

  // Rotation should be R_z(π/2)
  Eigen::Matrix3d R_expected;
  R_expected << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Matrix3d R_actual = T.block<3, 3>(0, 0);
  EXPECT_NEAR((R_actual - R_expected).norm(), 0.0, kLooseTol);

  // Translation should be zero
  Eigen::Vector3d t_actual = T.block<3, 1>(0, 3);
  EXPECT_NEAR(t_actual.norm(), 0.0, kTol);
}

TEST(SE3, ExpPureTranslation) {
  // When ω = 0, exp should give a pure translation
  Eigen::Matrix<double, 6, 1> xi;
  xi << 0.0, 0.0, 0.0, 1.0, 2.0, 3.0;

  Eigen::Matrix4d T = expSE3(xi);

  // Rotation should be identity
  Eigen::Matrix3d R_actual = T.block<3, 3>(0, 0);
  EXPECT_NEAR((R_actual - Eigen::Matrix3d::Identity()).norm(), 0.0, kTol);

  // Translation should be v (since J_l(0) = I)
  EXPECT_NEAR(T(0, 3), 1.0, kTol);
  EXPECT_NEAR(T(1, 3), 2.0, kTol);
  EXPECT_NEAR(T(2, 3), 3.0, kTol);
}

// =============================================================================
// log: basic properties
// =============================================================================

TEST(SE3, LogIdentityIsZero) {
  Eigen::Matrix<double, 6, 1> xi = logSE3(Eigen::Matrix4d::Identity());
  EXPECT_NEAR(xi.norm(), 0.0, kTol);
}

TEST(SE3, ExpLogRoundtrip) {
  // log(exp(ξ)) = ξ  for small enough angles
  std::mt19937 rng(123);
  for (int i = 0; i < 100; ++i) {
    Eigen::Matrix<double, 6, 1> xi_orig = randomTwist(rng, M_PI - 0.01, 5.0);
    Eigen::Matrix4d T = expSE3(xi_orig);
    Eigen::Matrix<double, 6, 1> xi_recovered = logSE3(T);

    EXPECT_NEAR((xi_orig - xi_recovered).norm(), 0.0, kLooseTol)
        << "Failed for xi = " << xi_orig.transpose();
  }
}

TEST(SE3, LogExpRoundtrip) {
  // exp(log(T)) = T
  std::mt19937 rng(456);
  for (int i = 0; i < 100; ++i) {
    Eigen::Matrix4d T_orig = randomSE3(rng);
    Eigen::Matrix<double, 6, 1> xi = logSE3(T_orig);
    Eigen::Matrix4d T_recovered = expSE3(xi);

    EXPECT_NEAR((T_orig - T_recovered).norm(), 0.0, kLooseTol)
        << "Failed for T:\n"
        << T_orig;
  }
}

// =============================================================================
// inverse
// =============================================================================

TEST(SE3, InverseProduct) {
  // T * T^{-1} = I
  std::mt19937 rng(789);
  for (int i = 0; i < 50; ++i) {
    Eigen::Matrix4d T = randomSE3(rng);
    Eigen::Matrix4d T_inv = inverseSE3(T);
    Eigen::Matrix4d product = T * T_inv;

    EXPECT_NEAR((product - Eigen::Matrix4d::Identity()).norm(), 0.0, kLooseTol);
  }
}

TEST(SE3, InverseIsValidSE3) {
  std::mt19937 rng(321);
  for (int i = 0; i < 50; ++i) {
    Eigen::Matrix4d T = randomSE3(rng);
    Eigen::Matrix4d T_inv = inverseSE3(T);
    EXPECT_TRUE(isValidSE3(T_inv));
  }
}

// =============================================================================
// transformPoint
// =============================================================================

TEST(SE3, TransformPointIdentity) {
  Eigen::Vector3d p(1.0, 2.0, 3.0);
  Eigen::Vector3d p_transformed =
      transformPoint(Eigen::Matrix4d::Identity(), p);
  EXPECT_NEAR((p - p_transformed).norm(), 0.0, kTol);
}

TEST(SE3, TransformPointPureTranslation) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = 10.0;
  T(1, 3) = 20.0;
  T(2, 3) = 30.0;

  Eigen::Vector3d p(1.0, 2.0, 3.0);
  Eigen::Vector3d expected(11.0, 22.0, 33.0);

  Eigen::Vector3d result = transformPoint(T, p);
  EXPECT_NEAR((result - expected).norm(), 0.0, kTol);
}

TEST(SE3, TransformPointConsistency) {
  // transformPoint(T, p) should equal R*p + t
  std::mt19937 rng(654);
  for (int i = 0; i < 50; ++i) {
    Eigen::Matrix4d T = randomSE3(rng);
    Eigen::Vector3d p(rng() / 1e9, rng() / 1e9, rng() / 1e9);

    Eigen::Vector3d result = transformPoint(T, p);
    Eigen::Vector3d expected = T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);

    EXPECT_NEAR((result - expected).norm(), 0.0, kTol);
  }
}

// =============================================================================
// Adjoint
// =============================================================================

TEST(SE3, AdjointIdentity) {
  Eigen::Matrix<double, 6, 6> Ad = adjointSE3(Eigen::Matrix4d::Identity());
  using Mat6 = Eigen::Matrix<double, 6, 6>;
  EXPECT_NEAR((Ad - Mat6::Identity()).norm(), 0.0, kTol);
}

TEST(SE3, AdjointTransformsTwists) {
  // The adjoint should satisfy: T · exp(ξ^) · T^{-1} = exp((Ad_T · ξ)^)
  // Equivalently: T · exp(ξ^) = exp((Ad_T · ξ)^) · T
  std::mt19937 rng(111);
  for (int i = 0; i < 30; ++i) {
    Eigen::Matrix4d T = randomSE3(rng);
    Eigen::Matrix<double, 6, 1> xi = randomTwist(rng, 0.5, 2.0);

    Eigen::Matrix<double, 6, 6> Ad = adjointSE3(T);
    Eigen::Matrix<double, 6, 1> Ad_xi = Ad * xi;

    Eigen::Matrix4d lhs = T * expSE3(xi) * inverseSE3(T);
    Eigen::Matrix4d rhs = expSE3(Ad_xi);

    EXPECT_NEAR((lhs - rhs).norm(), 0.0, kLooseTol)
        << "Failed for T:\n"
        << T << "\nxi = " << xi.transpose();
  }
}

// =============================================================================
// Composition consistency: exp(a) * exp(b) should be a valid SE(3) element
// =============================================================================

TEST(SE3, CompositionIsValid) {
  std::mt19937 rng(222);
  for (int i = 0; i < 50; ++i) {
    Eigen::Matrix4d T1 = randomSE3(rng);
    Eigen::Matrix4d T2 = randomSE3(rng);
    Eigen::Matrix4d T12 = T1 * T2;
    EXPECT_TRUE(isValidSE3(T12));
  }
}

// =============================================================================
// SE(3) exp rotation part should match SO(3) exp
// =============================================================================

TEST(SE3, ExpRotationMatchesSO3) {
  std::mt19937 rng(333);
  for (int i = 0; i < 50; ++i) {
    Eigen::Matrix<double, 6, 1> xi = randomTwist(rng, M_PI - 0.01, 5.0);
    Eigen::Vector3d w = xi.head<3>();

    Eigen::Matrix4d T = expSE3(xi);
    Eigen::Matrix3d R_se3 = T.block<3, 3>(0, 0);
    Eigen::Matrix3d R_so3 = expSO3(w);

    EXPECT_NEAR((R_se3 - R_so3).norm(), 0.0, kTol);
  }
}

}  // namespace
}  // namespace lie
}  // namespace backend
}  // namespace substral
