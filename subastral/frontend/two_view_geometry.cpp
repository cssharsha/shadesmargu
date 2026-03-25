#include "subastral/frontend/two_view_geometry.hpp"

#include <cmath>
#include <iostream>
#include <opencv2/calib3d.hpp>

namespace substral {
namespace frontend {

TwoViewResult TwoViewGeometry::estimate(const std::vector<cv::KeyPoint>& kp1,
                                        const std::vector<cv::KeyPoint>& kp2,
                                        const std::vector<cv::DMatch>& matches,
                                        const CameraIntrinsics& intrinsics,
                                        double ransac_threshold,
                                        double ransac_confidence,
                                        double min_triangulation_angle) {
  TwoViewResult result;

  if (matches.size() < 8) {
    std::cerr << "TwoViewGeometry: too few matches (" << matches.size()
              << " < 8)" << std::endl;
    return result;
  }

  // Extract matched point coordinates
  std::vector<cv::Point2f> pts1, pts2;
  std::vector<int> match_indices;
  pts1.reserve(matches.size());
  pts2.reserve(matches.size());
  match_indices.reserve(matches.size());

  for (size_t i = 0; i < matches.size(); ++i) {
    pts1.push_back(kp1[matches[i].queryIdx].pt);
    pts2.push_back(kp2[matches[i].trainIdx].pt);
    match_indices.push_back(static_cast<int>(i));
  }

  // Undistort points
  cv::Mat K = intrinsics.cameraMatrix();
  cv::Mat dist = intrinsics.distCoeffs();
  std::vector<cv::Point2f> pts1_undist, pts2_undist;
  cv::undistortPoints(pts1, pts1_undist, K, dist, cv::noArray(), K);
  cv::undistortPoints(pts2, pts2_undist, K, dist, cv::noArray(), K);

  // Find essential matrix with RANSAC
  cv::Mat inlier_mask;
  cv::Mat E =
      cv::findEssentialMat(pts1_undist, pts2_undist, K, cv::RANSAC,
                           ransac_confidence, ransac_threshold, inlier_mask);

  if (E.empty()) {
    std::cerr << "TwoViewGeometry: findEssentialMat failed" << std::endl;
    return result;
  }

  // Count inliers
  result.num_inliers = cv::countNonZero(inlier_mask);
  if (result.num_inliers < 8) {
    std::cerr << "TwoViewGeometry: too few inliers (" << result.num_inliers
              << " < 8)" << std::endl;
    return result;
  }

  // Recover pose (R, t) from essential matrix
  cv::Mat R_cv, t_cv;
  int good_pts =
      cv::recoverPose(E, pts1_undist, pts2_undist, K, R_cv, t_cv, inlier_mask);

  if (good_pts < 8) {
    std::cerr << "TwoViewGeometry: recoverPose returned too few good points ("
              << good_pts << " < 8)" << std::endl;
    return result;
  }

  // Convert cv::Mat to Eigen (manual — avoids opencv2/core/eigen.hpp which
  // pulls in cuda_runtime.h via Eigen's unsupported Tensor module)
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) result.R(r, c) = R_cv.at<double>(r, c);

  Eigen::Vector3d t_eigen(t_cv.at<double>(0), t_cv.at<double>(1),
                          t_cv.at<double>(2));
  result.t = t_eigen.normalized();

  // Triangulate points using inliers
  // Projection matrices: P1 = K[I|0], P2 = K[R|t]
  cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64F);
  K.copyTo(P1(cv::Rect(0, 0, 3, 3)));

  cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64F);
  cv::Mat KR = K * R_cv;
  cv::Mat Kt = K * t_cv;
  KR.copyTo(P2(cv::Rect(0, 0, 3, 3)));
  Kt.copyTo(P2(cv::Rect(3, 0, 1, 3)));

  // Collect inlier points for triangulation
  std::vector<cv::Point2f> inlier_pts1, inlier_pts2;
  std::vector<int> inlier_match_indices;
  for (int i = 0; i < static_cast<int>(pts1_undist.size()); ++i) {
    if (inlier_mask.at<uchar>(i)) {
      inlier_pts1.push_back(pts1_undist[i]);
      inlier_pts2.push_back(pts2_undist[i]);
      inlier_match_indices.push_back(match_indices[i]);
    }
  }

  if (inlier_pts1.empty()) {
    return result;
  }

  cv::Mat points4d;
  cv::triangulatePoints(P1, P2, inlier_pts1, inlier_pts2, points4d);

  // Filter and store valid 3D points
  filterTriangulatedPoints(points4d, result.R, result.t, K, inlier_pts1,
                           inlier_pts2, inlier_mask, inlier_match_indices,
                           result);

  result.success = !result.points_3d.empty();
  return result;
}

void TwoViewGeometry::filterTriangulatedPoints(
    const cv::Mat& points4d, const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
    const cv::Mat& K, const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2, const cv::Mat& /*inlier_mask*/,
    const std::vector<int>& match_indices, TwoViewResult& result) {
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  constexpr double kMaxReprojError = 4.0;   // pixels
  constexpr double kMinDepth = 0.01;
  constexpr double kMaxDepth = 100.0;
  constexpr double kMinParallaxDeg = 0.5;  // reject very low-parallax points

  auto& stats = result.filter_stats;
  stats.total_candidates = points4d.cols;

  // Camera centers for parallax computation
  // cam1 at origin, cam2 at -R^T * t
  Eigen::Vector3d C2 = -R.transpose() * t;

  for (int i = 0; i < points4d.cols; ++i) {
    // Convert from homogeneous
    double w = points4d.at<float>(3, i);
    if (std::abs(w) < 1e-10) {
      ++stats.rejected_w_zero;
      continue;
    }

    Eigen::Vector3d pt3d(points4d.at<float>(0, i) / w,
                         points4d.at<float>(1, i) / w,
                         points4d.at<float>(2, i) / w);

    // Check depth in camera 1 (z > 0)
    if (pt3d.z() < kMinDepth || pt3d.z() > kMaxDepth) {
      ++stats.rejected_depth_cam1;
      continue;
    }

    // Check depth in camera 2
    Eigen::Vector3d pt_cam2 = R * pt3d + t;
    if (pt_cam2.z() < kMinDepth || pt_cam2.z() > kMaxDepth) {
      ++stats.rejected_depth_cam2;
      continue;
    }

    // Check reprojection error in camera 1
    double u1_proj = fx * pt3d.x() / pt3d.z() + cx;
    double v1_proj = fy * pt3d.y() / pt3d.z() + cy;
    double err1 = std::sqrt(std::pow(u1_proj - pts1[i].x, 2) +
                            std::pow(v1_proj - pts1[i].y, 2));
    if (err1 > kMaxReprojError) {
      ++stats.rejected_reproj_cam1;
      continue;
    }

    // Check reprojection error in camera 2
    double u2_proj = fx * pt_cam2.x() / pt_cam2.z() + cx;
    double v2_proj = fy * pt_cam2.y() / pt_cam2.z() + cy;
    double err2 = std::sqrt(std::pow(u2_proj - pts2[i].x, 2) +
                            std::pow(v2_proj - pts2[i].y, 2));
    if (err2 > kMaxReprojError) {
      ++stats.rejected_reproj_cam2;
      continue;
    }

    // Parallax angle: angle between rays from cam1 and cam2 to this point
    Eigen::Vector3d ray1 = pt3d.normalized();
    Eigen::Vector3d ray2 = (pt3d - C2).normalized();
    double cos_parallax = ray1.dot(ray2);
    double parallax = std::acos(std::max(-1.0, std::min(1.0, cos_parallax))) *
                       180.0 / M_PI;
    if (parallax < kMinParallaxDeg) {
      ++stats.rejected_low_parallax;
      continue;
    }

    // Record stats for accepted point
    stats.reproj_err1.push_back(err1);
    stats.reproj_err2.push_back(err2);
    stats.depth_cam1.push_back(pt3d.z());
    stats.depth_cam2.push_back(pt_cam2.z());
    stats.parallax_deg.push_back(parallax);
    ++stats.accepted;

    result.points_3d.push_back(pt3d);
    result.inlier_indices.push_back(match_indices[i]);
  }
}

}  // namespace frontend
}  // namespace substral
