#pragma once

// =============================================================================
// VFEVisualizer — Rerun-based visualization for Visual Front-End
//
// Logs estimated camera trajectory, ground truth, triangulated 3D map points,
// and feature matches to a Rerun recording.
//
// Entity hierarchy:
//   /world/trajectory/estimated    — Estimated camera trajectory
//   /world/trajectory/ground_truth — Ground truth trajectory
//   /world/map_points              — Triangulated 3D points
//   /world/cameras/cam_N           — Camera frustums at keyframe poses
//   /images/matches                — Feature match visualization
// =============================================================================

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "subastral/loader/tum_loader.h"
#include "subastral/types/frame.h"
#include "subastral/types/map_point.h"
#include "viz/rerun/ba_visualizer.hpp"  // for VizConfig

namespace substral {
namespace rerun_viz {

/// Visual Front-End visualizer using Rerun.
class VFEVisualizer {
 public:
  VFEVisualizer() = default;
  ~VFEVisualizer();

  /// Initialize the recording stream. Returns false on failure.
  bool init(const VizConfig& config);

  /// Log ground-truth trajectory as a 3D line strip.
  void logGroundTruth(const std::vector<loader::GroundTruthPose>& gt);

  /// Log keyframe camera positions as trajectory + frustums.
  void logKeyframes(const std::vector<Frame>& keyframes);

  /// Log triangulated 3D map points.
  void logMapPoints(const std::unordered_map<int, MapPoint>& points);

  /// Log depth-sensor-based ground-truth 3D points (for comparison).
  void logDepthPoints(const std::vector<Eigen::Vector3d>& points);

  /// Set the frame timeline index for subsequent log calls.
  void setFrame(int frame_id);

  /// Log only newly added map points (highlighted in a distinct color).
  void logNewMapPoints(const std::vector<Eigen::Vector3d>& points, int frame_id);

  /// Log feature matches between two frames as an image overlay.
  void logMatches(const std::string& label, const Frame& frame1,
                  const Frame& frame2, const std::vector<cv::DMatch>& matches,
                  const std::vector<int>& inlier_indices, const cv::Mat& img1,
                  const cv::Mat& img2);

 private:
  struct Impl;
  Impl* impl_ = nullptr;
};

}  // namespace rerun_viz
}  // namespace substral
