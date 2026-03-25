#pragma once

#include <string>

#include "subastral/loader/dataset_types.h"

namespace substral {
namespace loader {

// =============================================================================
// EuRoC / TUM-VI dataset loader
// =============================================================================
//
// Parses sequences in the EuRoC MAV format, used by:
//   - TUM-VI (https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset)
//   - EuRoC MAV (https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
//
// Directory structure:
//   mav0/
//     cam0/
//       data.csv          — timestamp[ns], filename
//       data/             — grayscale PNG images
//       sensor.yaml       — camera intrinsics (optional)
//     cam1/               — second camera (optional, same structure)
//     imu0/
//       data.csv          — timestamp[ns], gx, gy, gz, ax, ay, az
//       sensor.yaml       — IMU noise parameters (optional)
//     state_groundtruth_estimate0/
//       data.csv          — timestamp[ns], tx,ty,tz, qw,qx,qy,qz, ...
//
// Key format differences from TUM RGB-D:
//   - Timestamps in nanoseconds (converted to seconds internally)
//   - GT quaternion is w-first (qw,qx,qy,qz)
//   - IMU has full 6-axis (gyro + accel)
//   - CSV with header row (first line starting with #)
// =============================================================================

class EuRoCLoader {
 public:
  bool load(const std::string& sequence_dir);
  const VisionDataset& dataset() const { return dataset_; }
  void printStats() const;

 private:
  bool parseImageCSV(const std::string& filepath,
                     std::vector<ImageEntry>& entries,
                     const std::string& data_dir);
  bool parseIMU(const std::string& filepath,
                std::vector<ImuMeasurement>& measurements);
  bool parseGroundTruth(const std::string& filepath,
                        std::vector<GroundTruthPose>& poses);
  void detectIntrinsics(const std::string& sequence_dir);

  VisionDataset dataset_;
};

}  // namespace loader
}  // namespace substral
