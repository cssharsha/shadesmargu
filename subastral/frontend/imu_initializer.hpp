#pragma once

#include <vector>

#include "subastral/loader/tum_loader.h"
#include "subastral/types/camera.h"
#include "subastral/types/frame.h"
#include "subastral/types/map.h"

namespace substral {
namespace frontend {

/// IMU-aided initialization for monocular SLAM.
///
/// Uses IMU pre-integration between frames to:
///   1. Estimate metric baseline between two views
///   2. Provide rotation prior for essential matrix decomposition
///   3. Select initialization frames with sufficient parallax
///
/// Requires full 6-axis IMU (accelerometer + gyroscope).
/// TUM RGB-D accelerometer-only data is NOT sufficient.
///
/// Target datasets: EuRoC MAV, TUM-VI.
///
/// STATUS: Interface only. Pre-integration not yet implemented.
///         Returns false from initialize(), causing fallback to monocular.
class ImuInitializer {
 public:
  struct Config {
    double min_parallax_deg;
    int max_frame_gap;
    double gravity_magnitude;
    Config() : min_parallax_deg(5.0), max_frame_gap(30), gravity_magnitude(9.81) {}
  };

  explicit ImuInitializer(const Config& config = Config()) : config_(config) {}

  /// Attempt IMU-aided initialization.
  /// Returns true on success, false to signal fallback to monocular.
  bool initialize(const std::vector<loader::ImuMeasurement>& imu_data,
                  std::vector<Frame>& frames,
                  const CameraIntrinsics& intrinsics, Map& map, int& out_idx1,
                  int& out_idx2);

 private:
  Config config_;
};

}  // namespace frontend
}  // namespace substral
