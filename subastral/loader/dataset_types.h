#pragma once

#include <string>
#include <vector>

#include "subastral/types/camera.h"

namespace substral {
namespace loader {

/// A single timestamped image entry.
struct ImageEntry {
  double timestamp;
  std::string relative_path;  // e.g., "rgb/1305031102.175304.png"
};

/// A single ground-truth pose.
struct GroundTruthPose {
  double timestamp;
  double tx, ty, tz;
  double qx, qy, qz, qw;  // Hamilton quaternion, w last in storage
};

/// A single 6-axis IMU measurement (accelerometer + gyroscope).
struct ImuMeasurement {
  double timestamp;
  double ax, ay, az;  // accelerometer [m/s^2]
  double gx, gy, gz;  // gyroscope [rad/s] (zero if unavailable)
};

/// Generic vision dataset — produced by any loader (TUM RGB-D, EuRoC, etc.)
struct VisionDataset {
  CameraIntrinsics intrinsics;
  std::vector<ImageEntry> rgb_frames;
  std::vector<ImageEntry> depth_frames;  // empty if no depth sensor
  std::vector<GroundTruthPose> ground_truth;
  std::vector<ImuMeasurement> imu_data;
  bool has_full_imu = false;  // true only if gyroscope data is present
  std::string base_path;      // absolute path to sequence directory
};

}  // namespace loader
}  // namespace substral
