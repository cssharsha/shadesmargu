#pragma once

#include <Eigen/Core>
#include <vector>

#include "subastral/loader/dataset_types.h"

namespace substral {
namespace frontend {

/// IMU pre-integration following Forster et al. (2017).
///
/// Accumulates gyroscope and accelerometer measurements between two
/// timestamps to produce a single pre-integrated measurement encoding
/// relative rotation, velocity, and position — all in the body frame
/// of the starting pose (independent of absolute pose/velocity/gravity).
///
/// Usage:
///   ImuPreintegrator preint;
///   preint.integrateRange(imu_data, t_keyframe_i, t_keyframe_j);
///   auto result = preint.result();
///   // result.delta_R, delta_v, delta_p encode relative motion
class ImuPreintegrator {
 public:
  struct Result {
    Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d delta_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d delta_p = Eigen::Vector3d::Zero();
    double dt = 0.0;  // total integration time

    // Bias Jacobians (for first-order correction without re-integrating)
    Eigen::Matrix3d d_R_d_bg = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d d_v_d_bg = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d d_v_d_ba = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d d_p_d_bg = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d d_p_d_ba = Eigen::Matrix3d::Zero();
  };

  explicit ImuPreintegrator(
      const Eigen::Vector3d& bias_gyro = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& bias_accel = Eigen::Vector3d::Zero());

  /// Integrate a single step with given dt, gyro and accel readings.
  /// Bias-corrected internally.
  void integrate(double dt, const Eigen::Vector3d& gyro,
                 const Eigen::Vector3d& accel);

  /// Integrate between two consecutive IMU measurements using midpoint method.
  void integrateMeasurement(const loader::ImuMeasurement& m0,
                            const loader::ImuMeasurement& m1);

  /// Integrate all IMU measurements between t_start and t_end.
  /// Handles boundary interpolation for timestamps not on IMU samples.
  void integrateRange(const std::vector<loader::ImuMeasurement>& imu_data,
                      double t_start, double t_end);

  void reset();
  void setBiases(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba);

  const Result& result() const { return result_; }

  /// First-order bias correction: given updated biases, return corrected
  /// pre-integrated measurement without re-integrating.
  Result corrected(const Eigen::Vector3d& bg_new,
                   const Eigen::Vector3d& ba_new) const;

 private:
  Eigen::Vector3d bias_gyro_;
  Eigen::Vector3d bias_accel_;
  Result result_;
};

}  // namespace frontend
}  // namespace substral
