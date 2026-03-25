// Standalone diagnostic: analyze two-view geometry triangulation quality
// against ground truth on TUM fr1/xyz.
//
// Dumps per-point: depth (unit-baseline), parallax, reprojection error,
// and compares triangulated depth against GT depth from known poses.
//
// Build: bazel build //subastral/frontend:two_view_diagnostic
// Run:   bazel run //subastral/frontend:two_view_diagnostic

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

#include "subastral/frontend/feature_extractor_cpu.hpp"
#include "subastral/frontend/feature_matcher_cpu.hpp"
#include "subastral/frontend/two_view_geometry.hpp"
#include "subastral/loader/tum_loader.h"

using namespace substral;

// Compute T_world_camera from a GT pose
static Eigen::Matrix4d poseToMatrix(const loader::GroundTruthPose& p) {
  Eigen::Quaterniond q(p.qw, p.qx, p.qy, p.qz);
  q.normalize();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
  T(0, 3) = p.tx;
  T(1, 3) = p.ty;
  T(2, 3) = p.tz;
  return T;
}

static const loader::GroundTruthPose& findNearestGT(
    const std::vector<loader::GroundTruthPose>& gt, double timestamp) {
  auto it = std::lower_bound(gt.begin(), gt.end(), timestamp,
                             [](const loader::GroundTruthPose& p, double t) {
                               return p.timestamp < t;
                             });
  if (it == gt.end()) return gt.back();
  if (it == gt.begin()) return *it;
  auto prev = std::prev(it);
  return (std::abs(prev->timestamp - timestamp) <
          std::abs(it->timestamp - timestamp))
             ? *prev
             : *it;
}

struct Stats {
  double min, max, mean, median;
  static Stats compute(std::vector<double> v) {
    if (v.empty()) return {0, 0, 0, 0};
    std::sort(v.begin(), v.end());
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    return {v.front(), v.back(), sum / v.size(), v[v.size() / 2]};
  }
};

static void printStats(const char* label, const std::vector<double>& v) {
  auto s = Stats::compute(std::vector<double>(v));
  std::cout << "  " << std::left << std::setw(25) << label << std::right
            << " min=" << std::fixed << std::setprecision(4) << s.min
            << " max=" << s.max << " mean=" << s.mean << " median=" << s.median
            << "  (n=" << v.size() << ")" << std::endl;
}

int main() {
  const std::string kDatasetPath =
      "/data/se/tum_datasets/rgbd_dataset_freiburg1_xyz";

  // --- Load dataset ---
  loader::TUMLoader loader;
  if (!loader.load(kDatasetPath)) {
    std::cerr << "Failed to load dataset" << std::endl;
    return 1;
  }
  auto dataset = loader.dataset();
  loader.printStats();

  // --- Test multiple frame gaps ---
  const std::vector<int> gaps = {5, 10, 15, 20, 30};

  for (int gap : gaps) {
    int idx1 = 0, idx2 = gap;
    if (idx2 >= static_cast<int>(dataset.rgb_frames.size())) continue;

    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "Frame pair: " << idx1 << " - " << idx2
              << " (gap=" << gap << ")" << std::endl;
    std::cout << std::string(70, '=') << std::endl;

    // GT poses
    double ts1 = dataset.rgb_frames[idx1].timestamp;
    double ts2 = dataset.rgb_frames[idx2].timestamp;
    Eigen::Matrix4d T_w_c1 = poseToMatrix(findNearestGT(dataset.ground_truth, ts1));
    Eigen::Matrix4d T_w_c2 = poseToMatrix(findNearestGT(dataset.ground_truth, ts2));

    Eigen::Vector3d gt_pos1 = T_w_c1.block<3, 1>(0, 3);
    Eigen::Vector3d gt_pos2 = T_w_c2.block<3, 1>(0, 3);
    double gt_baseline = (gt_pos2 - gt_pos1).norm();

    // GT relative rotation
    Eigen::Matrix3d R_gt_rel = (T_w_c1.block<3, 3>(0, 0).transpose() *
                                 T_w_c2.block<3, 3>(0, 0));
    double gt_rot_trace = R_gt_rel.trace();
    double gt_rot_deg =
        std::acos(std::max(-1.0, std::min(1.0, (gt_rot_trace - 1.0) / 2.0))) *
        180.0 / M_PI;

    std::cout << "  GT baseline:    " << std::fixed << std::setprecision(4)
              << gt_baseline << " m" << std::endl;
    std::cout << "  GT rotation:    " << gt_rot_deg << " deg" << std::endl;
    std::cout << "  GT cam1 pos:    [" << gt_pos1.transpose() << "]" << std::endl;
    std::cout << "  GT cam2 pos:    [" << gt_pos2.transpose() << "]" << std::endl;
    std::cout << "  GT translation: ["
              << (gt_pos2 - gt_pos1).normalized().transpose() << "]"
              << std::endl;

    // --- Extract features and match ---
    std::string path1 =
        dataset.base_path + "/" + dataset.rgb_frames[idx1].relative_path;
    std::string path2 =
        dataset.base_path + "/" + dataset.rgb_frames[idx2].relative_path;

    cv::Mat img1 = cv::imread(path1, cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(path2, cv::IMREAD_GRAYSCALE);
    if (img1.empty() || img2.empty()) {
      std::cerr << "  Failed to load images" << std::endl;
      continue;
    }

    frontend::FeatureExtractorCPU extractor(1000);
    frontend::FeatureMatcherCPU matcher;

    auto r1 = extractor.extract(img1);
    auto r2 = extractor.extract(img2);
    auto matches = matcher.match(r1.descriptors, r2.descriptors, 0.75f);

    std::cout << "  Features:       " << r1.keypoints.size() << " / "
              << r2.keypoints.size() << std::endl;
    std::cout << "  Matches:        " << matches.size() << std::endl;

    // --- Two-view geometry ---
    frontend::TwoViewGeometry tvg;
    auto tvr = tvg.estimate(r1.keypoints, r2.keypoints, matches,
                            dataset.intrinsics);

    if (!tvr.success) {
      std::cerr << "  Two-view geometry FAILED" << std::endl;
      continue;
    }

    // Estimated rotation angle
    double est_trace = tvr.R.trace();
    double est_rot_deg =
        std::acos(std::max(-1.0, std::min(1.0, (est_trace - 1.0) / 2.0))) *
        180.0 / M_PI;

    std::cout << "  Est rotation:   " << std::fixed << std::setprecision(2)
              << est_rot_deg << " deg (GT: " << gt_rot_deg << " deg)"
              << std::endl;
    std::cout << "  Est translation:[" << std::fixed << std::setprecision(4)
              << tvr.t.transpose() << "]" << std::endl;
    std::cout << "  GT trans (norm):[" << std::fixed << std::setprecision(4)
              << (gt_pos2 - gt_pos1).normalized().transpose() << "]"
              << std::endl;

    // Compare translation directions.
    // recoverPose gives [R|t] where p_cam2 = R * p_cam1 + t.
    // Cam2 center in cam1 frame: C2 = -R^T * t  (this is the cam1→cam2 direction)
    // GT cam1→cam2 direction in cam1 frame: R_w_c1^T * (pos2 - pos1)
    Eigen::Matrix3d R_w_c1 = T_w_c1.block<3, 3>(0, 0);
    Eigen::Vector3d gt_t_cam1 = R_w_c1.transpose() * (gt_pos2 - gt_pos1);
    Eigen::Vector3d est_cam1_to_cam2 = -tvr.R.transpose() * tvr.t;
    double t_dot = est_cam1_to_cam2.normalized().dot(gt_t_cam1.normalized());
    std::cout << "  Est cam1→cam2:  [" << std::fixed << std::setprecision(4)
              << est_cam1_to_cam2.normalized().transpose() << "]" << std::endl;
    std::cout << "  GT  cam1→cam2:  [" << std::fixed << std::setprecision(4)
              << gt_t_cam1.normalized().transpose() << "]" << std::endl;
    std::cout << "  Direction dot:   " << std::fixed << std::setprecision(4)
              << t_dot << " (1.0 = perfect)" << std::endl;

    std::cout << "  Inliers:        " << tvr.num_inliers << std::endl;
    std::cout << "  Triangulated:   " << tvr.points_3d.size() << std::endl;

    // --- Filter stats ---
    const auto& fs = tvr.filter_stats;
    std::cout << "\n  Filter breakdown:" << std::endl;
    std::cout << "    Candidates:      " << fs.total_candidates << std::endl;
    std::cout << "    Rej w~0:         " << fs.rejected_w_zero << std::endl;
    std::cout << "    Rej depth cam1:  " << fs.rejected_depth_cam1 << std::endl;
    std::cout << "    Rej depth cam2:  " << fs.rejected_depth_cam2 << std::endl;
    std::cout << "    Rej reproj cam1: " << fs.rejected_reproj_cam1 << std::endl;
    std::cout << "    Rej reproj cam2: " << fs.rejected_reproj_cam2 << std::endl;
    std::cout << "    Accepted:        " << fs.accepted << std::endl;

    if (fs.reproj_err1.empty()) continue;

    std::cout << "\n  Accepted point stats:" << std::endl;
    printStats("Reproj err cam1 (px)", fs.reproj_err1);
    printStats("Reproj err cam2 (px)", fs.reproj_err2);
    printStats("Depth cam1 (unit bl)", fs.depth_cam1);
    printStats("Depth cam2 (unit bl)", fs.depth_cam2);
    printStats("Parallax (deg)", fs.parallax_deg);

    // --- Cross-check: compare unit-baseline depth with GT depth ---
    // GT depth for a point: transform to cam1 frame using GT pose, take z
    // Unit-baseline depth: tvr.points_3d[i].z()
    // Ratio should be consistent: gt_depth = unit_depth * gt_baseline
    std::cout << "\n  --- Depth cross-check against GT ---" << std::endl;

    // T_cam1_world = inv(T_world_cam1)
    Eigen::Matrix4d T_c1_w = T_w_c1.inverse();
    Eigen::Matrix4d T_c2_w = T_w_c2.inverse();

    // The estimated T_cam2_cam1 = [R|t] (recoverPose convention)
    // Points are in cam1 frame (from triangulatePoints with P1=K[I|0])
    // So GT depth of the same physical point in cam1 frame would be
    // at a scale factor of gt_baseline (since ||t_est|| = 1)

    std::vector<double> depth_ratios;
    std::vector<double> gt_reproj_err1, gt_reproj_err2;

    cv::Mat K = dataset.intrinsics.cameraMatrix();
    double fx = K.at<double>(0, 0), fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2), cy = K.at<double>(1, 2);

    // For each triangulated point, compute:
    // 1. Expected GT depth = unit_depth * gt_baseline
    // 2. Reproject into cam1 using GT pose to see if geometry is consistent
    int n_dump = std::min(20, static_cast<int>(tvr.points_3d.size()));
    std::cout << "  First " << n_dump << " points (unit-bl depth, "
              << "scaled depth=depth*" << std::fixed << std::setprecision(4)
              << gt_baseline << "):" << std::endl;
    std::cout << "    " << std::setw(5) << "idx"
              << std::setw(10) << "depth_ub"
              << std::setw(10) << "depth_m"
              << std::setw(10) << "reproj1"
              << std::setw(10) << "reproj2"
              << std::setw(10) << "parallax"
              << std::endl;

    for (int i = 0; i < static_cast<int>(tvr.points_3d.size()); ++i) {
      const auto& pt = tvr.points_3d[i];
      double depth_ub = pt.z();
      double depth_m = depth_ub * gt_baseline;

      // Reproject: point is in cam1 frame at unit-baseline scale
      // Scale to metric, transform to world, then to cam1/cam2 via GT
      Eigen::Vector3d p_cam1_metric = pt * gt_baseline;
      Eigen::Vector3d p_world = T_w_c1.block<3, 3>(0, 0) * p_cam1_metric +
                                 T_w_c1.block<3, 1>(0, 3);

      // Project into cam1 via GT
      Eigen::Vector3d p_c1_gt =
          T_c1_w.block<3, 3>(0, 0) * p_world + T_c1_w.block<3, 1>(0, 3);
      double u1 = fx * p_c1_gt.x() / p_c1_gt.z() + cx;
      double v1 = fy * p_c1_gt.y() / p_c1_gt.z() + cy;

      // Project into cam2 via GT
      Eigen::Vector3d p_c2_gt =
          T_c2_w.block<3, 3>(0, 0) * p_world + T_c2_w.block<3, 1>(0, 3);
      double u2 = fx * p_c2_gt.x() / p_c2_gt.z() + cx;
      double v2 = fy * p_c2_gt.y() / p_c2_gt.z() + cy;

      // Get the original pixel observations
      int mi = tvr.inlier_indices[i];
      cv::Point2f obs1 = r1.keypoints[matches[mi].queryIdx].pt;
      cv::Point2f obs2 = r2.keypoints[matches[mi].trainIdx].pt;

      double err1 = std::sqrt(std::pow(u1 - obs1.x, 2) +
                               std::pow(v1 - obs1.y, 2));
      double err2 = std::sqrt(std::pow(u2 - obs2.x, 2) +
                               std::pow(v2 - obs2.y, 2));

      gt_reproj_err1.push_back(err1);
      gt_reproj_err2.push_back(err2);

      if (i < n_dump) {
        std::cout << "    " << std::setw(5) << i
                  << std::setw(10) << std::fixed << std::setprecision(3) << depth_ub
                  << std::setw(10) << depth_m
                  << std::setw(10) << err1
                  << std::setw(10) << err2
                  << std::setw(10) << fs.parallax_deg[i]
                  << std::endl;
      }
    }

    std::cout << "\n  GT-scaled reprojection error:" << std::endl;
    printStats("GT reproj cam1 (px)", gt_reproj_err1);
    printStats("GT reproj cam2 (px)", gt_reproj_err2);

    // Depth distribution histogram (unit-baseline)
    std::cout << "\n  Depth histogram (unit-baseline):" << std::endl;
    std::vector<int> hist(20, 0);
    for (const auto& pt : tvr.points_3d) {
      int bin = std::min(19, static_cast<int>(pt.z()));
      if (bin >= 0) ++hist[bin];
    }
    for (int b = 0; b < 20; ++b) {
      if (hist[b] > 0) {
        std::cout << "    [" << std::setw(3) << b << "-" << std::setw(3)
                  << b + 1 << "): " << std::string(hist[b], '#')
                  << " (" << hist[b] << ")" << std::endl;
      }
    }

    // Parallax histogram
    std::cout << "\n  Parallax histogram (degrees):" << std::endl;
    std::vector<int> par_hist(20, 0);
    for (double p : fs.parallax_deg) {
      int bin = std::min(19, static_cast<int>(p * 2));  // 0.5 deg bins
      if (bin >= 0) ++par_hist[bin];
    }
    for (int b = 0; b < 20; ++b) {
      if (par_hist[b] > 0) {
        double lo = b * 0.5, hi = (b + 1) * 0.5;
        std::cout << "    [" << std::fixed << std::setprecision(1)
                  << lo << "-" << hi << "): "
                  << std::string(std::min(60, par_hist[b]), '#')
                  << " (" << par_hist[b] << ")" << std::endl;
      }
    }
  }

  return 0;
}
