#pragma once

#include <Eigen/Core>
#include <vector>

#include "subastral/loader/dataset_types.h"

namespace substral {
namespace frontend {

/// Estimates gravity direction from initial static IMU accelerometer data.
///
/// When the device is stationary, the accelerometer reads the reaction
/// to gravity. Averaging these readings gives the gravity direction in
/// the IMU body frame. This is used to align the world frame (Z-up)
/// with gravity.
class GravityInitializer {
 public:
  struct Config {
    int num_samples;
    double gravity_magnitude;
    double max_accel_variance;  // reject if device is moving
    Config()
        : num_samples(200),
          gravity_magnitude(9.81),
          max_accel_variance(0.5) {}
  };

  struct Result {
    Eigen::Vector3d gravity_imu;    // gravity direction in IMU frame (unit)
    Eigen::Matrix3d R_world_imu;    // rotation: IMU frame → Z-up world
    double gravity_magnitude;       // estimated |g|
    bool success = false;
  };

  explicit GravityInitializer(const Config& config = Config())
      : config_(config) {}

  /// Estimate gravity from the first N accelerometer readings.
  Result estimate(const std::vector<loader::ImuMeasurement>& imu_data) const;

 private:
  Config config_;
};

}  // namespace frontend
}  // namespace substral
