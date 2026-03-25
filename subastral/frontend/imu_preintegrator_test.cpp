#include "subastral/frontend/imu_preintegrator.hpp"

#include <Eigen/Geometry>
#include <cmath>

#include "gtest/gtest.h"
#include "subastral/backend/lie/so3.hpp"

namespace substral {
namespace frontend {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

constexpr double kTol = 1e-6;

void expectMatrixNear(const Matrix3d& a, const Matrix3d& b, double tol = kTol) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      EXPECT_NEAR(a(r, c), b(r, c), tol) << "at (" << r << "," << c << ")";
}

void expectVectorNear(const Vector3d& a, const Vector3d& b, double tol = kTol) {
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(a(i), b(i), tol) << "at " << i;
}

TEST(ImuPreintegratorTest, ZeroInput) {
  ImuPreintegrator preint;

  // 100 steps of zero gyro and zero accel
  for (int i = 0; i < 100; ++i) {
    preint.integrate(0.005, Vector3d::Zero(), Vector3d::Zero());
  }

  auto r = preint.result();
  expectMatrixNear(r.delta_R, Matrix3d::Identity());
  expectVectorNear(r.delta_v, Vector3d::Zero());
  expectVectorNear(r.delta_p, Vector3d::Zero());
  EXPECT_NEAR(r.dt, 0.5, 1e-10);
}

TEST(ImuPreintegratorTest, ConstantGyroZ) {
  // Constant angular velocity about Z: omega = (0, 0, pi/2)
  // After 1 second, should rotate 90 degrees about Z
  ImuPreintegrator preint;
  Vector3d omega(0, 0, M_PI / 2);

  int steps = 200;
  double dt = 1.0 / steps;
  for (int i = 0; i < steps; ++i) {
    preint.integrate(dt, omega, Vector3d::Zero());
  }

  auto r = preint.result();
  Matrix3d expected_R = backend::lie::expSO3(Vector3d(0, 0, M_PI / 2));
  expectMatrixNear(r.delta_R, expected_R, 1e-4);

  // No acceleration → velocity and position should be zero
  expectVectorNear(r.delta_v, Vector3d::Zero(), 1e-4);
  expectVectorNear(r.delta_p, Vector3d::Zero(), 1e-4);
}

TEST(ImuPreintegratorTest, ConstantAccelX) {
  // Constant acceleration along X: a = (1, 0, 0), no rotation
  // After 1 second: v = (1, 0, 0), p = (0.5, 0, 0)
  ImuPreintegrator preint;
  Vector3d accel(1.0, 0, 0);

  int steps = 200;
  double dt = 1.0 / steps;
  for (int i = 0; i < steps; ++i) {
    preint.integrate(dt, Vector3d::Zero(), accel);
  }

  auto r = preint.result();
  expectMatrixNear(r.delta_R, Matrix3d::Identity());
  expectVectorNear(r.delta_v, Vector3d(1.0, 0, 0), 1e-4);
  expectVectorNear(r.delta_p, Vector3d(0.5, 0, 0), 1e-4);
}

TEST(ImuPreintegratorTest, ConstantAccelWithRotation) {
  // Rotate about Z at 1 rad/s while accelerating along body X at 1 m/s^2.
  // After 1s, rotation angle = 1 rad.
  // Velocity and position are more complex due to rotating body frame.
  ImuPreintegrator preint;
  Vector3d omega(0, 0, 1.0);
  Vector3d accel(1.0, 0, 0);

  int steps = 1000;
  double dt = 1.0 / steps;
  for (int i = 0; i < steps; ++i) {
    preint.integrate(dt, omega, accel);
  }

  auto r = preint.result();

  // Rotation should be exp((0,0,1))
  Matrix3d expected_R = backend::lie::expSO3(Vector3d(0, 0, 1.0));
  expectMatrixNear(r.delta_R, expected_R, 1e-3);

  // Velocity magnitude should be 1.0 (constant accel for 1s)
  EXPECT_NEAR(r.delta_v.norm(), 1.0, 0.05);

  // Position should be non-zero
  EXPECT_GT(r.delta_p.norm(), 0.3);
}

TEST(ImuPreintegratorTest, Reset) {
  ImuPreintegrator preint;
  preint.integrate(0.01, Vector3d(1, 2, 3), Vector3d(4, 5, 6));

  preint.reset();
  auto r = preint.result();
  expectMatrixNear(r.delta_R, Matrix3d::Identity());
  expectVectorNear(r.delta_v, Vector3d::Zero());
  expectVectorNear(r.delta_p, Vector3d::Zero());
  EXPECT_NEAR(r.dt, 0.0, 1e-15);
}

TEST(ImuPreintegratorTest, IntegrateMeasurement) {
  // Verify midpoint integration matches manual
  ImuPreintegrator preint;

  loader::ImuMeasurement m0{1.0, 1.0, 0, 0, 0, 0, 0.1};
  loader::ImuMeasurement m1{1.01, 1.0, 0, 0, 0, 0, 0.1};
  preint.integrateMeasurement(m0, m1);

  auto r = preint.result();
  EXPECT_NEAR(r.dt, 0.01, 1e-10);
  EXPECT_GT(r.delta_p.norm(), 0);
}

TEST(ImuPreintegratorTest, IntegrateRange) {
  // Create synthetic IMU data: constant accel along X
  std::vector<loader::ImuMeasurement> imu_data;
  for (int i = 0; i <= 200; ++i) {
    double t = 10.0 + i * 0.005;  // 200Hz starting at t=10s
    imu_data.push_back({t, 2.0, 0, 0, 0, 0, 0});
  }

  ImuPreintegrator preint;
  preint.integrateRange(imu_data, 10.0, 11.0);

  auto r = preint.result();
  EXPECT_NEAR(r.dt, 1.0, 0.01);
  expectVectorNear(r.delta_v, Vector3d(2.0, 0, 0), 0.05);
  expectVectorNear(r.delta_p, Vector3d(1.0, 0, 0), 0.05);
}

TEST(ImuPreintegratorTest, IntegrateRangeSubset) {
  // Integrate only a subset of the IMU data
  std::vector<loader::ImuMeasurement> imu_data;
  for (int i = 0; i <= 400; ++i) {
    double t = 10.0 + i * 0.005;
    imu_data.push_back({t, 1.0, 0, 0, 0, 0, 0});
  }

  ImuPreintegrator preint;
  preint.integrateRange(imu_data, 10.25, 10.75);

  auto r = preint.result();
  EXPECT_NEAR(r.dt, 0.5, 0.01);
  expectVectorNear(r.delta_v, Vector3d(0.5, 0, 0), 0.05);
  expectVectorNear(r.delta_p, Vector3d(0.125, 0, 0), 0.02);
}

TEST(ImuPreintegratorTest, BiasJacobianNumerical) {
  // Verify that corrected() approximation matches re-integration
  // for small bias perturbation
  Vector3d bg0(0.01, -0.005, 0.003);
  Vector3d ba0(0.1, -0.05, 0.02);

  // Integrate with original biases
  ImuPreintegrator preint0(bg0, ba0);
  Vector3d omega(0.5, 0.1, -0.3);
  Vector3d accel(0.5, 9.81, 0.2);

  int steps = 100;
  double dt = 0.005;
  for (int i = 0; i < steps; ++i) {
    preint0.integrate(dt, omega, accel);
  }

  // Perturb biases slightly
  Vector3d delta_bg(0.001, -0.001, 0.0005);
  Vector3d delta_ba(0.01, -0.005, 0.002);
  Vector3d bg1 = bg0 + delta_bg;
  Vector3d ba1 = ba0 + delta_ba;

  // First-order correction
  auto corrected = preint0.corrected(bg1, ba1);

  // Full re-integration with new biases
  ImuPreintegrator preint1(bg1, ba1);
  for (int i = 0; i < steps; ++i) {
    preint1.integrate(dt, omega, accel);
  }

  auto reintegrated = preint1.result();

  // Velocity and position should match to first order
  // (error should be O(||delta_b||^2))
  double v_err = (corrected.delta_v - reintegrated.delta_v).norm();
  double p_err = (corrected.delta_p - reintegrated.delta_p).norm();

  EXPECT_LT(v_err, 0.01) << "Velocity correction error too large";
  EXPECT_LT(p_err, 0.001) << "Position correction error too large";

  // Rotation correction should also be close
  Matrix3d R_err =
      corrected.delta_R.transpose() * reintegrated.delta_R;
  Vector3d rot_err = backend::lie::logSO3(R_err);
  EXPECT_LT(rot_err.norm(), 0.001) << "Rotation correction error too large";
}

}  // namespace
}  // namespace frontend
}  // namespace substral
