#include "subastral/frontend/imu_preintegrator.hpp"

#include <algorithm>
#include <cmath>

#include "subastral/backend/lie/so3.hpp"

namespace substral {
namespace frontend {

using Eigen::Matrix3d;
using Eigen::Vector3d;

ImuPreintegrator::ImuPreintegrator(const Vector3d& bias_gyro,
                                   const Vector3d& bias_accel)
    : bias_gyro_(bias_gyro), bias_accel_(bias_accel) {}

void ImuPreintegrator::integrate(double dt, const Vector3d& gyro,
                                 const Vector3d& accel) {
  if (dt <= 0) return;

  // Bias-corrected measurements
  Vector3d omega = gyro - bias_gyro_;
  Vector3d acc = accel - bias_accel_;

  // Incremental rotation
  Vector3d omega_dt = omega * dt;
  Matrix3d dR = backend::lie::expSO3(omega_dt);
  Matrix3d Jr = backend::lie::rightJacobianSO3(omega_dt);

  // Save current state before update (needed for Jacobians and integration)
  Matrix3d R_prev = result_.delta_R;

  // Update pre-integrated quantities (order matters: p, v, then R)
  // p uses current v and R; v uses current R; R updates last
  result_.delta_p += result_.delta_v * dt + 0.5 * R_prev * acc * dt * dt;
  result_.delta_v += R_prev * acc * dt;
  result_.delta_R = R_prev * dR;
  result_.dt += dt;

  // Update bias Jacobians (Forster supplementary B.7-B.9)
  Matrix3d acc_hat = backend::lie::hat(acc);

  result_.d_p_d_ba += result_.d_v_d_ba * dt - 0.5 * R_prev * dt * dt;
  result_.d_p_d_bg +=
      result_.d_v_d_bg * dt - 0.5 * R_prev * acc_hat * result_.d_R_d_bg * dt * dt;

  result_.d_v_d_ba -= R_prev * dt;
  result_.d_v_d_bg -= R_prev * acc_hat * result_.d_R_d_bg * dt;

  result_.d_R_d_bg = dR.transpose() * result_.d_R_d_bg - Jr * dt;
}

void ImuPreintegrator::integrateMeasurement(const loader::ImuMeasurement& m0,
                                            const loader::ImuMeasurement& m1) {
  double dt = m1.timestamp - m0.timestamp;
  if (dt <= 0) return;

  // Midpoint integration: average gyro and accel between samples
  Vector3d gyro_mid(0.5 * (m0.gx + m1.gx), 0.5 * (m0.gy + m1.gy),
                    0.5 * (m0.gz + m1.gz));
  Vector3d accel_mid(0.5 * (m0.ax + m1.ax), 0.5 * (m0.ay + m1.ay),
                     0.5 * (m0.az + m1.az));

  integrate(dt, gyro_mid, accel_mid);
}

// Linearly interpolate an IMU measurement at time t between m0 and m1
static loader::ImuMeasurement interpolate(const loader::ImuMeasurement& m0,
                                          const loader::ImuMeasurement& m1,
                                          double t) {
  double alpha = (t - m0.timestamp) / (m1.timestamp - m0.timestamp);
  alpha = std::max(0.0, std::min(1.0, alpha));

  loader::ImuMeasurement m;
  m.timestamp = t;
  m.ax = m0.ax + alpha * (m1.ax - m0.ax);
  m.ay = m0.ay + alpha * (m1.ay - m0.ay);
  m.az = m0.az + alpha * (m1.az - m0.az);
  m.gx = m0.gx + alpha * (m1.gx - m0.gx);
  m.gy = m0.gy + alpha * (m1.gy - m0.gy);
  m.gz = m0.gz + alpha * (m1.gz - m0.gz);
  return m;
}

void ImuPreintegrator::integrateRange(
    const std::vector<loader::ImuMeasurement>& imu_data, double t_start,
    double t_end) {
  if (imu_data.empty() || t_start >= t_end) return;

  // Binary search for the first IMU sample >= t_start
  auto it = std::lower_bound(
      imu_data.begin(), imu_data.end(), t_start,
      [](const loader::ImuMeasurement& m, double t) {
        return m.timestamp < t;
      });

  if (it == imu_data.end()) return;
  if (it == imu_data.begin() && it->timestamp > t_start) {
    // t_start is before all IMU data — start from first sample
  }

  // Back up one sample for interpolation at t_start
  auto start_it = (it != imu_data.begin()) ? std::prev(it) : it;

  // Collect relevant IMU samples between t_start and t_end,
  // with interpolated boundary samples
  std::vector<loader::ImuMeasurement> segment;

  // Interpolated start sample
  if (start_it->timestamp < t_start && std::next(start_it) != imu_data.end()) {
    segment.push_back(interpolate(*start_it, *std::next(start_it), t_start));
  } else {
    segment.push_back(*start_it);
  }

  // Interior samples
  for (auto sit = it; sit != imu_data.end() && sit->timestamp < t_end; ++sit) {
    if (sit->timestamp > t_start) {
      segment.push_back(*sit);
    }
  }

  // Interpolated end sample
  if (!segment.empty() && segment.back().timestamp < t_end) {
    // Find the sample pair bracketing t_end
    auto end_it = std::lower_bound(
        imu_data.begin(), imu_data.end(), t_end,
        [](const loader::ImuMeasurement& m, double t) {
          return m.timestamp < t;
        });
    if (end_it != imu_data.begin()) {
      auto prev_end = std::prev(end_it);
      if (end_it != imu_data.end() && prev_end->timestamp < t_end) {
        segment.push_back(interpolate(*prev_end, *end_it, t_end));
      }
    }
  }

  // Integrate consecutive pairs
  for (size_t i = 0; i + 1 < segment.size(); ++i) {
    integrateMeasurement(segment[i], segment[i + 1]);
  }
}

void ImuPreintegrator::reset() { result_ = Result{}; }

void ImuPreintegrator::setBiases(const Vector3d& bg, const Vector3d& ba) {
  bias_gyro_ = bg;
  bias_accel_ = ba;
}

ImuPreintegrator::Result ImuPreintegrator::corrected(
    const Vector3d& bg_new, const Vector3d& ba_new) const {
  Vector3d delta_bg = bg_new - bias_gyro_;
  Vector3d delta_ba = ba_new - bias_accel_;

  Result corrected_result = result_;
  corrected_result.delta_R =
      result_.delta_R * backend::lie::expSO3(result_.d_R_d_bg * delta_bg);
  corrected_result.delta_v +=
      result_.d_v_d_bg * delta_bg + result_.d_v_d_ba * delta_ba;
  corrected_result.delta_p +=
      result_.d_p_d_bg * delta_bg + result_.d_p_d_ba * delta_ba;

  return corrected_result;
}

}  // namespace frontend
}  // namespace substral
