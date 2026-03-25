#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <vector>

#include "subastral/types/camera.h"

namespace substral {
namespace frontend {

/// Result of two-view geometry estimation.
struct TwoViewResult {
  Eigen::Matrix3d R =
      Eigen::Matrix3d::Identity();  // Rotation from view 1 to view 2
  Eigen::Vector3d t = Eigen::Vector3d::Zero();  // Unit-norm translation

  std::vector<Eigen::Vector3d>
      points_3d;  // Triangulated 3D points (in frame 1 coords)
  std::vector<int> inlier_indices;  // Indices into the input matches

  int num_inliers = 0;
  bool success = false;

  // Diagnostic stats from triangulation filtering
  struct FilterStats {
    int total_candidates = 0;
    int rejected_w_zero = 0;
    int rejected_depth_cam1 = 0;
    int rejected_depth_cam2 = 0;
    int rejected_reproj_cam1 = 0;
    int rejected_reproj_cam2 = 0;
    int rejected_low_parallax = 0;
    int accepted = 0;

    // Reprojection errors for accepted points
    std::vector<double> reproj_err1;
    std::vector<double> reproj_err2;
    // Depths for accepted points
    std::vector<double> depth_cam1;
    std::vector<double> depth_cam2;
    // Parallax angles in degrees
    std::vector<double> parallax_deg;
  } filter_stats;
};

/// Two-view geometry estimation: essential matrix, pose recovery,
/// triangulation.
///
/// All operations run on CPU using OpenCV's findEssentialMat, recoverPose,
/// and triangulatePoints. These are inherently sequential (RANSAC) and
/// operate on small data, so GPU acceleration is not beneficial.
class TwoViewGeometry {
 public:
  /// Estimate relative pose and triangulate points from matched features.
  ///
  /// @param kp1 Keypoints in image 1 (pixel coordinates)
  /// @param kp2 Keypoints in image 2 (pixel coordinates)
  /// @param matches Good matches between the two sets
  /// @param intrinsics Camera intrinsics for undistortion and triangulation
  /// @param ransac_threshold RANSAC inlier threshold in pixels (default 1.0)
  /// @param ransac_confidence RANSAC confidence (default 0.999)
  /// @param min_triangulation_angle Minimum parallax angle in degrees
  /// (default 1.0)
  TwoViewResult estimate(const std::vector<cv::KeyPoint>& kp1,
                         const std::vector<cv::KeyPoint>& kp2,
                         const std::vector<cv::DMatch>& matches,
                         const CameraIntrinsics& intrinsics,
                         double ransac_threshold = 1.0,
                         double ransac_confidence = 0.999,
                         double min_triangulation_angle = 1.0);

 private:
  /// Filter triangulated points: positive depth, bounded reprojection error,
  /// reasonable depth range.
  void filterTriangulatedPoints(const cv::Mat& points4d,
                                const Eigen::Matrix3d& R,
                                const Eigen::Vector3d& t, const cv::Mat& K,
                                const std::vector<cv::Point2f>& pts1,
                                const std::vector<cv::Point2f>& pts2,
                                const cv::Mat& inlier_mask,
                                const std::vector<int>& match_indices,
                                TwoViewResult& result);
};

}  // namespace frontend
}  // namespace substral
