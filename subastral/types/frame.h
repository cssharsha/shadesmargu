#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace substral {

/// A single image frame with extracted features and estimated pose.
struct Frame {
  int id = -1;
  double timestamp = 0.0;
  std::string image_path;  // Absolute path to the RGB image

  // Extracted features (populated by FeatureExtractor)
  std::vector<cv::KeyPoint> keypoints;      // Pixel coordinates
  std::vector<cv::Point2f> keypoints_norm;  // Undistorted normalized coords
  cv::Mat descriptors;                      // Feature descriptors (CPU)

  // Map point associations (-1 if keypoint has no associated map point)
  std::vector<int> map_point_ids;

  // Camera pose in world frame: T_world_camera
  // Transforms points from camera frame to world frame.
  Eigen::Matrix4d T_world_camera = Eigen::Matrix4d::Identity();

  /// Number of keypoints with associated map points.
  int numTrackedPoints() const {
    int count = 0;
    for (int id : map_point_ids) {
      if (id >= 0) ++count;
    }
    return count;
  }
};

}  // namespace substral
