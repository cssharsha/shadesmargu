#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <utility>
#include <vector>

namespace substral {

/// A 3D map point observed by one or more frames.
struct MapPoint {
  int id = -1;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();

  // Representative descriptor for matching against new frames
  cv::Mat descriptor;

  // RGB color sampled from the source image at the keypoint location
  uint8_t r = 128, g = 128, b = 128;

  // List of (frame_id, keypoint_index) pairs that observe this point
  std::vector<std::pair<int, int>> observations;

  /// Number of frames that observe this point.
  int numObservations() const { return static_cast<int>(observations.size()); }
};

}  // namespace substral
