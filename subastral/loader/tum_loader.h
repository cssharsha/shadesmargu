#pragma once

#include <string>
#include <vector>

#include "subastral/loader/dataset_types.h"

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
// =============================================================================

/// Backwards compatibility alias.
using TUMDataset = VisionDataset;

class TUMLoader {
 public:
  bool load(const std::string& sequence_dir);
  const VisionDataset& dataset() const { return dataset_; }
  void printStats() const;

 private:
  bool parseImageList(const std::string& filepath,
                      std::vector<ImageEntry>& entries);
  bool parseGroundTruth(const std::string& filepath,
                        std::vector<GroundTruthPose>& poses);
  bool parseAccelerometer(const std::string& filepath,
                          std::vector<ImuMeasurement>& measurements);
  int detectSensor(const std::string& path);
  void setIntrinsics(int sensor_id);

  VisionDataset dataset_;
  int sensor_id_ = 0;
};

}  // namespace loader
}  // namespace substral
