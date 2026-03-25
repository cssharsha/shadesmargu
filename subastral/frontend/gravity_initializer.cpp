#include "subastral/frontend/gravity_initializer.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace substral {
namespace frontend {

using Eigen::Matrix3d;
using Eigen::Vector3d;

GravityInitializer::Result GravityInitializer::estimate(
    const std::vector<loader::ImuMeasurement>& imu_data) const {
  Result result;

  int n = std::min(config_.num_samples, static_cast<int>(imu_data.size()));
  if (n < 10) {
    std::cerr << "GravityInitializer: too few IMU samples (" << n << ")"
              << std::endl;
    return result;
  }

  // Average accelerometer readings
  Vector3d accel_sum = Vector3d::Zero();
  for (int i = 0; i < n; ++i) {
    accel_sum += Vector3d(imu_data[i].ax, imu_data[i].ay, imu_data[i].az);
  }
  Vector3d accel_mean = accel_sum / n;

  // Check variance: if device is moving, accelerometer readings vary a lot
  double var_sum = 0;
  for (int i = 0; i < n; ++i) {
    Vector3d a(imu_data[i].ax, imu_data[i].ay, imu_data[i].az);
    double diff = (a - accel_mean).squaredNorm();
    var_sum += diff;
  }
  double variance = var_sum / n;

  if (variance > config_.max_accel_variance) {
    std::cerr << "GravityInitializer: accel variance too high (" << variance
              << " > " << config_.max_accel_variance
              << "). Device may be moving." << std::endl;
    return result;
  }

  // When static, accelerometer reads the reaction to gravity.
  // In EuRoC/TUM-VI convention: a_measured ≈ -g_body (reaction force)
  // Actually in most IMU conventions, when static pointing up: a_z ≈ +9.81
  // The gravity vector in the body frame points in the direction of a_mean
  result.gravity_magnitude = accel_mean.norm();
  result.gravity_imu = accel_mean.normalized();

  std::cout << "  Gravity in IMU frame: [" << std::fixed << std::setprecision(4)
            << result.gravity_imu.transpose() << "] |g|="
            << result.gravity_magnitude << " m/s^2" << std::endl;

  // Compute R_world_imu that aligns gravity with world -Z
  // (gravity points down in Z-up world)
  //
  // We want: R_world_imu * g_imu_dir = [0, 0, 1]
  // (because g_imu points "up" when measured by accelerometer on a table)
  //
  // Find rotation from g_imu_dir to z_world = [0, 0, 1]
  Vector3d z_world(0, 0, 1);
  Vector3d g_dir = result.gravity_imu;

  // Rotation from g_dir to z_world via Rodrigues
  Vector3d v = g_dir.cross(z_world);
  double c = g_dir.dot(z_world);

  if (c > 0.9999) {
    // Already aligned
    result.R_world_imu = Matrix3d::Identity();
  } else if (c < -0.9999) {
    // 180 degree rotation — find any perpendicular axis
    Vector3d perp = (std::abs(g_dir.x()) < 0.9) ? Vector3d(1, 0, 0)
                                                  : Vector3d(0, 1, 0);
    Vector3d axis = g_dir.cross(perp).normalized();
    // 180 deg rotation about axis: R = 2*axis*axis^T - I
    result.R_world_imu = 2.0 * axis * axis.transpose() - Matrix3d::Identity();
  } else {
    // General case: Rodrigues from cross product
    Matrix3d vx;
    vx << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    result.R_world_imu =
        Matrix3d::Identity() + vx + vx * vx * (1.0 / (1.0 + c));
  }

  result.success = true;
  return result;
}

}  // namespace frontend
}  // namespace substral
