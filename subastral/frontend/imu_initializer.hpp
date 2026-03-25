#pragma once

#include <memory>
#include <vector>

#include "subastral/frontend/gravity_initializer.hpp"
#include "subastral/frontend/imu_preintegrator.hpp"
#include "subastral/loader/dataset_types.h"
#include "subastral/types/camera.h"
#include "subastral/types/frame.h"
#include "subastral/types/map.h"
#include "subastral/types/transform_tree.h"

namespace substral {
namespace frontend {

/// IMU-aided initialization for monocular SLAM.
///
/// Uses IMU pre-integration to:
///   1. Estimate gravity direction → align world Z-up
///   2. Estimate metric baseline between initialization frames
///   3. Provide rotation prior for essential matrix decomposition
///   4. Select initialization frame pair with sufficient baseline
///
/// Requires full 6-axis IMU (accelerometer + gyroscope).
class ImuInitializer {
 public:
  struct Config {
    double min_baseline_m;       // minimum displacement for init (meters)
    double gravity_magnitude;
    int max_frame_gap;
    Config()
        : min_baseline_m(0.05),
          gravity_magnitude(9.81),
          max_frame_gap(50) {}
  };

  struct InitResult {
    int idx1 = -1;
    int idx2 = -1;
    Eigen::Matrix3d R_imu_12 = Eigen::Matrix3d::Identity();  // gyro rotation
    double rotation_deg = 0.0;
    bool success = false;
  };

  explicit ImuInitializer(const Config& config = Config())
      : config_(config) {}

  /// IMU-aided initialization.
  /// Estimates gravity, selects frame pair with sufficient baseline,
  /// registers transforms in the tree.
  InitResult initialize(
      const std::vector<loader::ImuMeasurement>& imu_data,
      const std::vector<loader::ImageEntry>& rgb_frames,
      const CameraIntrinsics& intrinsics,
      std::shared_ptr<TransformTree> tf_tree);

 private:
  Config config_;
  GravityInitializer gravity_init_;
};

}  // namespace frontend
}  // namespace substral
