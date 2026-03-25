#pragma once

#include <string>
#include <vector>

#include "subastral/types/camera.h"

namespace substral {
namespace loader {

// =============================================================================
// TUM RGB-D Benchmark dataset loader
// =============================================================================
//
// Parses TUM RGB-D sequences from:
//   https://cvg.cit.tum.de/data/datasets/rgbd-dataset
//
// Each sequence directory contains:
//   rgb.txt          — timestamp + relative path to RGB image
//   depth.txt        — timestamp + relative path to depth image
//   groundtruth.txt  — timestamp tx ty tz qx qy qz qw
//   accelerometer.txt — timestamp ax ay az
//   rgb/             — PNG images (640x480)
//   depth/           — 16-bit depth images (640x480)
//
// File format for rgb.txt / depth.txt:
//   First 3 lines are comments (start with #)
//   Each subsequent line: <timestamp> <relative_path>
//
// Ground truth format:
//   First 3 lines are comments (start with #)
//   Each subsequent line: <timestamp> <tx> <ty> <tz> <qx> <qy> <qz> <qw>
//   Quaternion is Hamilton convention with w last.
//
// Camera intrinsics are per-sensor (freiburg1/2/3), auto-detected from
// the directory name.
//
// =============================================================================

/// A single timestamped image entry from rgb.txt.
struct ImageEntry {
  double timestamp;
  std::string relative_path;  // e.g., "rgb/1305031102.175304.png"
};

/// A single ground-truth pose entry from groundtruth.txt.
struct GroundTruthPose {
  double timestamp;
  double tx, ty, tz;
  double qx, qy, qz, qw;  // Hamilton quaternion, w last
};

/// A single 6-axis IMU measurement (accelerometer + gyroscope).
/// TUM RGB-D only has accelerometer.txt (3-axis); gyro fields will be zero.
/// Full 6-axis IMU is available in EuRoC/TUM-VI datasets.
struct ImuMeasurement {
  double timestamp;
  double ax, ay, az;  // accelerometer [m/s^2]
  double gx, gy, gz;  // gyroscope [rad/s] (zero if unavailable)
};

/// Complete parsed TUM dataset.
struct TUMDataset {
  CameraIntrinsics intrinsics;
  std::vector<ImageEntry> rgb_frames;
  std::vector<ImageEntry> depth_frames;  // 16-bit PNG, scaled by 5000
  std::vector<GroundTruthPose> ground_truth;
  std::vector<ImuMeasurement> imu_data;  // 6-axis IMU (accel+gyro)
  bool has_full_imu = false;  // true only if gyroscope data is present
  std::string base_path;  // Absolute path to sequence directory
  int sensor_id = 0;      // 1=freiburg1, 2=freiburg2, 3=freiburg3
};

class TUMLoader {
 public:
  /// Load a TUM sequence from the given directory path.
  /// Returns true on success.
  bool load(const std::string& sequence_dir);

  /// Access the parsed dataset.
  const TUMDataset& dataset() const { return dataset_; }

  /// Print summary statistics.
  void printStats() const;

 private:
  /// Parse rgb.txt or depth.txt (same format).
  bool parseImageList(const std::string& filepath,
                      std::vector<ImageEntry>& entries);

  /// Parse groundtruth.txt.
  bool parseGroundTruth(const std::string& filepath,
                        std::vector<GroundTruthPose>& poses);

  /// Detect freiburg sensor (1, 2, or 3) from directory name.
  /// Returns 0 if unknown.
  int detectSensor(const std::string& path);

  /// Set intrinsics based on sensor ID.
  void setIntrinsics(int sensor_id);

  /// Parse accelerometer.txt (TUM RGB-D format: timestamp ax ay az).
  bool parseAccelerometer(const std::string& filepath,
                          std::vector<ImuMeasurement>& measurements);

  TUMDataset dataset_;
};

}  // namespace loader
}  // namespace substral
