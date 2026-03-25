#include "subastral/visual_frontend_pipeline.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "subastral/frontend/feature_extractor_cpu.hpp"
#include "subastral/frontend/feature_matcher_cpu.hpp"
#include "subastral/loader/tum_loader.h"

namespace substral {

// ---------------------------------------------------------------------------
Eigen::Matrix4d VisualFrontendPipeline::findNearestGTPose(
    const std::vector<loader::GroundTruthPose>& gt, double timestamp) {
  if (gt.empty()) return Eigen::Matrix4d::Identity();

  auto it = std::lower_bound(gt.begin(), gt.end(), timestamp,
                             [](const loader::GroundTruthPose& p, double t) {
                               return p.timestamp < t;
                             });

  const loader::GroundTruthPose* best = &gt.front();
  if (it != gt.end()) {
    best = &(*it);
    if (it != gt.begin()) {
      auto prev = std::prev(it);
      if (std::abs(prev->timestamp - timestamp) <
          std::abs(it->timestamp - timestamp)) {
        best = &(*prev);
      }
    }
  } else {
    best = &gt.back();
  }

  Eigen::Quaterniond q(best->qw, best->qx, best->qy, best->qz);
  q.normalize();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
  T(0, 3) = best->tx;
  T(1, 3) = best->ty;
  T(2, 3) = best->tz;
  return T;
}

// ---------------------------------------------------------------------------
VisualFrontendPipeline::VisualFrontendPipeline() {
  extractor_ = std::make_unique<frontend::FeatureExtractorCPU>(1000);
  matcher_ = std::make_unique<frontend::FeatureMatcherCPU>();
}

bool VisualFrontendPipeline::load(const std::string& sequence_dir) {
  loader::TUMLoader loader;
  if (!loader.load(sequence_dir)) return false;
  dataset_ = loader.dataset();
  loader.printStats();

  // Auto-detect available sensors
  sensor_config_.has_depth = !dataset_.depth_frames.empty();
  sensor_config_.has_imu = dataset_.has_full_imu;
  sensor_config_.has_accel_only =
      !dataset_.imu_data.empty() && !dataset_.has_full_imu;

  // Initialize transform tree with standard frame convention
  // Optical: X-right, Y-down, Z-forward (OpenCV)
  // World:   X-right, Y-forward, Z-up
  tf_tree_ = std::make_shared<TransformTree>();
  Eigen::Matrix4d T_world_optical = Eigen::Matrix4d::Identity();
  // clang-format off
  T_world_optical.block<3, 3>(0, 0) << 1,  0,  0,
                                        0,  0,  1,
                                        0, -1,  0;
  // clang-format on
  tf_tree_->setStatic("world", "optical", T_world_optical);

  return true;
}

// ---------------------------------------------------------------------------
std::string VisualFrontendPipeline::findAssociatedDepthPath(
    int rgb_frame_idx, double max_dt) const {
  if (dataset_.depth_frames.empty()) return "";

  double rgb_ts = dataset_.rgb_frames[rgb_frame_idx].timestamp;

  auto it = std::lower_bound(
      dataset_.depth_frames.begin(), dataset_.depth_frames.end(), rgb_ts,
      [](const loader::ImageEntry& e, double t) {
        return e.timestamp < t;
      });

  const loader::ImageEntry* best = nullptr;
  double best_dt = max_dt;

  if (it != dataset_.depth_frames.end()) {
    double dt = std::abs(it->timestamp - rgb_ts);
    if (dt < best_dt) {
      best = &(*it);
      best_dt = dt;
    }
  }
  if (it != dataset_.depth_frames.begin()) {
    auto prev = std::prev(it);
    double dt = std::abs(prev->timestamp - rgb_ts);
    if (dt < best_dt) {
      best = &(*prev);
    }
  }

  if (!best) return "";
  return dataset_.base_path + "/" + best->relative_path;
}

// ---------------------------------------------------------------------------
bool VisualFrontendPipeline::initializeFromDepth(
    int frame_idx, rerun_viz::VFEVisualizer* /*viz*/) {
  std::string rgb_path =
      dataset_.base_path + "/" + dataset_.rgb_frames[frame_idx].relative_path;
  cv::Mat img = cv::imread(rgb_path, cv::IMREAD_GRAYSCALE);
  if (img.empty()) return false;

  auto result = extractor_->extract(img);
  if (result.keypoints.size() < 50) return false;

  std::string depth_path = findAssociatedDepthPath(frame_idx);
  if (depth_path.empty()) {
    std::cerr << "  No depth image found for frame " << frame_idx << std::endl;
    return false;
  }

  Frame frame;
  frame.id = frame_idx;
  frame.timestamp = dataset_.rgb_frames[frame_idx].timestamp;
  frame.image_path = rgb_path;
  frame.keypoints = std::move(result.keypoints);
  frame.descriptors = std::move(result.descriptors);
  frame.T_world_camera = tf_tree_->lookup("world", "optical");

  int n_points =
      depth_init_.initialize(frame, depth_path, dataset_.intrinsics, map_);

  if (n_points < 50) {
    std::cerr << "  Depth init: only " << n_points
              << " valid points (need 50+)" << std::endl;
    return false;
  }

  map_.addKeyframe(std::move(frame));

  std::cout << "  Depth init: frame " << frame_idx << " | " << n_points
            << " 3D points from depth" << std::endl;

  return true;
}

// ---------------------------------------------------------------------------
bool VisualFrontendPipeline::initializeMonocular(
    rerun_viz::VFEVisualizer* viz) {
  constexpr int kRefFrame = 0;
  constexpr int kMinGap = 5;
  constexpr int kMaxGap = 50;
  constexpr int kMinPoints = 30;

  int n_frames = static_cast<int>(dataset_.rgb_frames.size());

  for (int gap = kMinGap; gap <= kMaxGap && kRefFrame + gap < n_frames;
       gap += 5) {
    int idx2 = kRefFrame + gap;
    std::cout << "  Trying pair (" << kRefFrame << ", " << idx2 << ")...";

    if (initialize(kRefFrame, idx2, viz) && map_.numPoints() >= kMinPoints) {
      std::cout << " success (" << map_.numPoints() << " points)" << std::endl;
      return true;
    }
    std::cout << " insufficient" << std::endl;
    map_.clear();
  }

  std::cerr << "  Monocular init: no valid pair found" << std::endl;
  return false;
}

// ==========================================================================
// initialize — two-view geometry from two frames
// ==========================================================================
bool VisualFrontendPipeline::initialize(int idx1, int idx2,
                                        rerun_viz::VFEVisualizer* viz) {
  std::string path1 =
      dataset_.base_path + "/" + dataset_.rgb_frames[idx1].relative_path;
  std::string path2 =
      dataset_.base_path + "/" + dataset_.rgb_frames[idx2].relative_path;

  cv::Mat img1 = cv::imread(path1, cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(path2, cv::IMREAD_GRAYSCALE);
  if (img1.empty() || img2.empty()) return false;

  auto r1 = extractor_->extract(img1);
  auto r2 = extractor_->extract(img2);

  auto matches = matcher_->match(r1.descriptors, r2.descriptors, 0.75f);
  if (matches.size() < 20) return false;

  auto tvr = two_view_.estimate(r1.keypoints, r2.keypoints, matches,
                                dataset_.intrinsics);
  if (!tvr.success) return false;

  double ts1 = dataset_.rgb_frames[idx1].timestamp;
  double ts2 = dataset_.rgb_frames[idx2].timestamp;

  // Unit baseline: world frame = cam1 frame (standard monocular convention)
  // T_cam1_cam2 = inv(T_cam2_cam1) = [R^T | -R^T * t]
  Eigen::Matrix4d T_cam1_cam2 = Eigen::Matrix4d::Identity();
  T_cam1_cam2.block<3, 3>(0, 0) = tvr.R.transpose();
  T_cam1_cam2.block<3, 1>(0, 3) = -tvr.R.transpose() * tvr.t;

  Eigen::Matrix4d T_world_optical = tf_tree_->lookup("world", "optical");
  Eigen::Matrix4d T_world_cam1 = T_world_optical;
  Eigen::Matrix4d T_world_cam2 = T_world_optical * T_cam1_cam2;

  // Create keyframes
  Frame f1;
  f1.id = idx1;
  f1.timestamp = ts1;
  f1.image_path = path1;
  f1.keypoints = std::move(r1.keypoints);
  f1.descriptors = std::move(r1.descriptors);
  f1.map_point_ids.resize(f1.keypoints.size(), -1);
  f1.T_world_camera = T_world_cam1;

  Frame f2;
  f2.id = idx2;
  f2.timestamp = ts2;
  f2.image_path = path2;
  f2.keypoints = std::move(r2.keypoints);
  f2.descriptors = std::move(r2.descriptors);
  f2.map_point_ids.resize(f2.keypoints.size(), -1);
  f2.T_world_camera = T_world_cam2;

  // Triangulate and add to map
  Eigen::Matrix3d R_wc1 = T_world_cam1.block<3, 3>(0, 0);
  Eigen::Vector3d t_wc1 = T_world_cam1.block<3, 1>(0, 3);

  for (size_t i = 0; i < tvr.points_3d.size(); ++i) {
    int mi = tvr.inlier_indices[i];
    int kp1 = matches[mi].queryIdx;
    int kp2 = matches[mi].trainIdx;

    Eigen::Vector3d p_world = R_wc1 * tvr.points_3d[i] + t_wc1;
    cv::Mat desc = f1.descriptors.row(kp1);
    int pid = map_.addMapPoint(p_world, desc);

    map_.addObservation(pid, f1.id, kp1);
    map_.addObservation(pid, f2.id, kp2);
    f1.map_point_ids[kp1] = pid;
    f2.map_point_ids[kp2] = pid;
  }

  map_.addKeyframe(std::move(f1));
  map_.addKeyframe(std::move(f2));

  // ---- Diagnostic logging ----
  std::cout << "  Init: frames " << idx1 << "," << idx2
            << " | matches=" << matches.size() << " inliers=" << tvr.num_inliers
            << " points=" << tvr.points_3d.size() << std::endl;

  // Rotation angle
  double trace = tvr.R.trace();
  double angle_rad =
      std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
  std::cout << "  Rotation:     " << std::fixed << std::setprecision(2)
            << angle_rad * 180.0 / M_PI << " deg" << std::endl;
  std::cout << "  Translation:  [" << std::fixed << std::setprecision(4)
            << tvr.t.transpose() << "]" << std::endl;

  // Filter rejection breakdown
  const auto& fs = tvr.filter_stats;
  std::cout << "\n  --- Triangulation Filter ---" << std::endl;
  std::cout << "  Candidates:        " << fs.total_candidates << std::endl;
  std::cout << "  Rejected w~0:      " << fs.rejected_w_zero << std::endl;
  std::cout << "  Rejected depth c1: " << fs.rejected_depth_cam1 << std::endl;
  std::cout << "  Rejected depth c2: " << fs.rejected_depth_cam2 << std::endl;
  std::cout << "  Rejected reproj c1:" << fs.rejected_reproj_cam1 << std::endl;
  std::cout << "  Rejected reproj c2:" << fs.rejected_reproj_cam2 << std::endl;
  std::cout << "  Accepted:          " << fs.accepted << std::endl;

  // Reprojection error stats
  if (!fs.reproj_err1.empty()) {
    auto stats = [](const std::vector<double>& v) {
      double sum = 0, mn = v[0], mx = v[0];
      for (double x : v) {
        sum += x;
        mn = std::min(mn, x);
        mx = std::max(mx, x);
      }
      std::vector<double> sorted = v;
      std::sort(sorted.begin(), sorted.end());
      double median = sorted[sorted.size() / 2];
      return std::make_tuple(mn, mx, sum / v.size(), median);
    };

    auto [min1, max1, mean1, med1] = stats(fs.reproj_err1);
    auto [min2, max2, mean2, med2] = stats(fs.reproj_err2);
    std::cout << "\n  --- Reprojection Error (accepted points) ---" << std::endl;
    std::cout << "  Cam1: min=" << std::fixed << std::setprecision(3)
              << min1 << " max=" << max1 << " mean=" << mean1
              << " median=" << med1 << " px" << std::endl;
    std::cout << "  Cam2: min=" << std::fixed << std::setprecision(3)
              << min2 << " max=" << max2 << " mean=" << mean2
              << " median=" << med2 << " px" << std::endl;

    auto [dmin1, dmax1, dmean1, dmed1] = stats(fs.depth_cam1);
    auto [dmin2, dmax2, dmean2, dmed2] = stats(fs.depth_cam2);
    std::cout << "\n  --- Depth (accepted points) ---" << std::endl;
    std::cout << "  Cam1: min=" << std::fixed << std::setprecision(3)
              << dmin1 << " max=" << dmax1 << " mean=" << dmean1
              << " median=" << dmed1 << std::endl;
    std::cout << "  Cam2: min=" << std::fixed << std::setprecision(3)
              << dmin2 << " max=" << dmax2 << " mean=" << dmean2
              << " median=" << dmed2 << std::endl;

    auto [pmin, pmax, pmean, pmed] = stats(fs.parallax_deg);
    std::cout << "\n  --- Parallax (accepted points) ---" << std::endl;
    std::cout << "  min=" << std::fixed << std::setprecision(3)
              << pmin << " max=" << pmax << " mean=" << pmean
              << " median=" << pmed << " deg" << std::endl;
  }

  // Independent reprojection verification: project map points into both frames
  // using the stored poses and intrinsics
  {
    const auto& kf1 = map_.keyframes()[map_.numKeyframes() - 2];
    const auto& kf2 = map_.keyframes()[map_.numKeyframes() - 1];
    cv::Mat K = dataset_.intrinsics.cameraMatrix();
    double fxv = K.at<double>(0, 0), fyv = K.at<double>(1, 1);
    double cxv = K.at<double>(0, 2), cyv = K.at<double>(1, 2);

    Eigen::Matrix4d T_cw1 = kf1.T_world_camera.inverse();
    Eigen::Matrix4d T_cw2 = kf2.T_world_camera.inverse();

    double sum_err1 = 0, sum_err2 = 0;
    int count = 0;

    for (const auto& [pid, mp] : map_.points()) {
      // Find observations in kf1 and kf2
      int kp1_idx = -1, kp2_idx = -1;
      for (const auto& [fid, kpi] : mp.observations) {
        if (fid == kf1.id) kp1_idx = kpi;
        if (fid == kf2.id) kp2_idx = kpi;
      }
      if (kp1_idx < 0 || kp2_idx < 0) continue;

      // Project into cam1
      Eigen::Vector3d p_c1 =
          T_cw1.block<3, 3>(0, 0) * mp.position + T_cw1.block<3, 1>(0, 3);
      double u1 = fxv * p_c1.x() / p_c1.z() + cxv;
      double v1 = fyv * p_c1.y() / p_c1.z() + cyv;
      double e1 = std::sqrt(std::pow(u1 - kf1.keypoints[kp1_idx].pt.x, 2) +
                            std::pow(v1 - kf1.keypoints[kp1_idx].pt.y, 2));

      // Project into cam2
      Eigen::Vector3d p_c2 =
          T_cw2.block<3, 3>(0, 0) * mp.position + T_cw2.block<3, 1>(0, 3);
      double u2 = fxv * p_c2.x() / p_c2.z() + cxv;
      double v2 = fyv * p_c2.y() / p_c2.z() + cyv;
      double e2 = std::sqrt(std::pow(u2 - kf2.keypoints[kp2_idx].pt.x, 2) +
                            std::pow(v2 - kf2.keypoints[kp2_idx].pt.y, 2));

      sum_err1 += e1;
      sum_err2 += e2;
      ++count;
    }

    if (count > 0) {
      std::cout << "\n  --- Independent Reproj Verification ---" << std::endl;
      std::cout << "  Points verified: " << count << std::endl;
      std::cout << "  Mean reproj cam1: " << std::fixed << std::setprecision(3)
                << sum_err1 / count << " px" << std::endl;
      std::cout << "  Mean reproj cam2: " << std::fixed << std::setprecision(3)
                << sum_err2 / count << " px" << std::endl;
    }
  }

  std::cout << std::endl;
  return true;
}

// ==========================================================================
// trackFrame — PnP tracking + new point triangulation
// ==========================================================================
bool VisualFrontendPipeline::trackFrame(int frame_idx,
                                        rerun_viz::VFEVisualizer* viz) {
  const Frame& prev_kf = map_.keyframes().back();

  std::string img_path =
      dataset_.base_path + "/" + dataset_.rgb_frames[frame_idx].relative_path;
  cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
  if (img.empty()) return false;

  // Extract features in new frame
  auto result = extractor_->extract(img);
  if (result.keypoints.size() < 20) return false;

  // Match against previous keyframe
  auto matches =
      matcher_->match(prev_kf.descriptors, result.descriptors, 0.75f);
  if (matches.size() < 20) return false;

  // ---- Separate matches into 3D-2D (for PnP) and 2D-2D (for triangulation)
  std::vector<int> match_with_3d;
  std::vector<int> match_without_3d;

  for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
    int prev_kp = matches[i].queryIdx;
    if (prev_kf.map_point_ids[prev_kp] >= 0) {
      match_with_3d.push_back(i);
    } else {
      match_without_3d.push_back(i);
    }
  }

  if (match_with_3d.size() < 10) return false;

  // ---- PnP pose estimation from 3D map points → 2D observations ----
  std::vector<cv::Point3f> obj_pts;
  std::vector<cv::Point2f> img_pts;
  obj_pts.reserve(match_with_3d.size());
  img_pts.reserve(match_with_3d.size());

  for (int mi : match_with_3d) {
    int prev_kp = matches[mi].queryIdx;
    int curr_kp = matches[mi].trainIdx;
    int pid = prev_kf.map_point_ids[prev_kp];

    const Eigen::Vector3d& pos = map_.getPoint(pid).position;
    obj_pts.emplace_back(static_cast<float>(pos.x()),
                         static_cast<float>(pos.y()),
                         static_cast<float>(pos.z()));
    img_pts.push_back(result.keypoints[curr_kp].pt);
  }

  cv::Mat K = dataset_.intrinsics.cameraMatrix();
  cv::Mat dist = dataset_.intrinsics.distCoeffs();
  cv::Mat rvec, tvec;
  cv::Mat pnp_inliers;

  bool ok = cv::solvePnPRansac(obj_pts, img_pts, K, dist, rvec, tvec,
                                false, 100, 8.0f, 0.99, pnp_inliers,
                                cv::SOLVEPNP_ITERATIVE);

  if (!ok || pnp_inliers.rows < 10) return false;

  // solvePnP returns T_camera_world: p_cam = R * p_world + t
  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);

  Eigen::Matrix4d T_camera_world = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      T_camera_world(r, c) = R_cv.at<double>(r, c);
  T_camera_world(0, 3) = tvec.at<double>(0);
  T_camera_world(1, 3) = tvec.at<double>(1);
  T_camera_world(2, 3) = tvec.at<double>(2);

  // T_world_camera = inv(T_camera_world)
  Eigen::Matrix3d R_cw = T_camera_world.block<3, 3>(0, 0);
  Eigen::Vector3d t_cw = T_camera_world.block<3, 1>(0, 3);
  Eigen::Matrix4d T_world_camera = Eigen::Matrix4d::Identity();
  T_world_camera.block<3, 3>(0, 0) = R_cw.transpose();
  T_world_camera.block<3, 1>(0, 3) = -R_cw.transpose() * t_cw;

  // ---- Create the new frame ----
  Frame new_frame;
  new_frame.id = frame_idx;
  new_frame.timestamp = dataset_.rgb_frames[frame_idx].timestamp;
  new_frame.image_path = img_path;
  new_frame.keypoints = std::move(result.keypoints);
  new_frame.descriptors = std::move(result.descriptors);
  new_frame.map_point_ids.resize(new_frame.keypoints.size(), -1);
  new_frame.T_world_camera = T_world_camera;

  // Carry forward map point associations for PnP inliers
  for (int i = 0; i < pnp_inliers.rows; ++i) {
    int inl_idx = pnp_inliers.at<int>(i);
    int match_idx = match_with_3d[inl_idx];
    int prev_kp = matches[match_idx].queryIdx;
    int curr_kp = matches[match_idx].trainIdx;
    int pid = prev_kf.map_point_ids[prev_kp];

    new_frame.map_point_ids[curr_kp] = pid;
    map_.addObservation(pid, new_frame.id, curr_kp);
  }

  // ---- Create new map points from matches without existing 3D ----
  int new_points = 0;
  std::vector<Eigen::Vector3d> new_point_positions;

  // Depth-first: if depth images are available, backproject directly
  if (sensor_config_.has_depth && !match_without_3d.empty()) {
    std::string depth_path = findAssociatedDepthPath(frame_idx);
    if (!depth_path.empty()) {
      cv::Mat depth_raw = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
      if (!depth_raw.empty() && depth_raw.type() == CV_16UC1) {
        Eigen::Matrix3d R_wc = T_world_camera.block<3, 3>(0, 0);
        Eigen::Vector3d t_wc = T_world_camera.block<3, 1>(0, 3);
        for (int mi : match_without_3d) {
          int curr_kp = matches[mi].trainIdx;
          int prev_kp = matches[mi].queryIdx;
          const cv::KeyPoint& kp = new_frame.keypoints[curr_kp];
          int u = static_cast<int>(std::round(kp.pt.x));
          int v = static_cast<int>(std::round(kp.pt.y));
          if (u < 0 || u >= depth_raw.cols || v < 0 || v >= depth_raw.rows)
            continue;
          uint16_t raw = depth_raw.at<uint16_t>(v, u);
          if (raw == 0) continue;
          double z = static_cast<double>(raw) / 5000.0;
          if (z < 0.1 || z > 10.0) continue;

          double x = (kp.pt.x - dataset_.intrinsics.cx) / dataset_.intrinsics.fx * z;
          double y = (kp.pt.y - dataset_.intrinsics.cy) / dataset_.intrinsics.fy * z;
          Eigen::Vector3d p_cam(x, y, z);
          Eigen::Vector3d p_world = R_wc * p_cam + t_wc;

          cv::Mat desc = new_frame.descriptors.row(curr_kp);
          int pid = map_.addMapPoint(p_world, desc);
          map_.addObservation(pid, prev_kf.id, prev_kp);
          map_.addObservation(pid, new_frame.id, curr_kp);
          map_.keyframes().back().map_point_ids[prev_kp] = pid;
          new_frame.map_point_ids[curr_kp] = pid;
          new_point_positions.push_back(p_world);
          ++new_points;
        }
      }
    }
  } else if (!match_without_3d.empty()) {
    Eigen::Matrix4d T_cw_prev = prev_kf.T_world_camera.inverse();
    Eigen::Matrix4d T_cw_curr = T_camera_world;  // already have it

    // P = K * T_camera_world(3x4)
    cv::Mat T_prev_34 = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat T_curr_34 = cv::Mat::zeros(3, 4, CV_64F);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 4; ++c) {
        T_prev_34.at<double>(r, c) = T_cw_prev(r, c);
        T_curr_34.at<double>(r, c) = T_cw_curr(r, c);
      }
    }
    cv::Mat P_prev = K * T_prev_34;
    cv::Mat P_curr = K * T_curr_34;

    // Collect 2D points for triangulation
    std::vector<cv::Point2f> tri_pts_prev, tri_pts_curr;
    std::vector<int> tri_match_indices;
    for (int mi : match_without_3d) {
      tri_pts_prev.push_back(prev_kf.keypoints[matches[mi].queryIdx].pt);
      tri_pts_curr.push_back(new_frame.keypoints[matches[mi].trainIdx].pt);
      tri_match_indices.push_back(mi);
    }

    cv::Mat pts4d;
    cv::triangulatePoints(P_prev, P_curr, tri_pts_prev, tri_pts_curr, pts4d);

    // Filter and add valid points
    Eigen::Matrix3d R_cw_curr_tri = T_cw_curr.block<3, 3>(0, 0);
    Eigen::Vector3d t_cw_curr_vec = T_cw_curr.block<3, 1>(0, 3);
    Eigen::Matrix3d R_cw_prev_e = T_cw_prev.block<3, 3>(0, 0);
    Eigen::Vector3d t_cw_prev_vec = T_cw_prev.block<3, 1>(0, 3);

    for (int i = 0; i < pts4d.cols; ++i) {
      float w = pts4d.at<float>(3, i);
      if (std::abs(w) < 1e-10f) continue;

      Eigen::Vector3d p_world(pts4d.at<float>(0, i) / w,
                              pts4d.at<float>(1, i) / w,
                              pts4d.at<float>(2, i) / w);

      // Check positive depth in both cameras
      Eigen::Vector3d p_prev_cam = R_cw_prev_e * p_world + t_cw_prev_vec;
      Eigen::Vector3d p_curr_cam = R_cw_curr_tri * p_world + t_cw_curr_vec;
      if (p_prev_cam.z() < 0.01 || p_curr_cam.z() < 0.01) continue;
      if (p_prev_cam.z() > 50.0 || p_curr_cam.z() > 50.0) continue;

      // Reprojection error check in current frame
      double u_proj =
          K.at<double>(0, 0) * p_curr_cam.x() / p_curr_cam.z() +
          K.at<double>(0, 2);
      double v_proj =
          K.at<double>(1, 1) * p_curr_cam.y() / p_curr_cam.z() +
          K.at<double>(1, 2);
      double err = std::sqrt(std::pow(u_proj - tri_pts_curr[i].x, 2) +
                             std::pow(v_proj - tri_pts_curr[i].y, 2));
      if (err > 4.0) continue;

      int mi = tri_match_indices[i];
      int prev_kp = matches[mi].queryIdx;
      int curr_kp = matches[mi].trainIdx;

      cv::Mat desc = new_frame.descriptors.row(curr_kp);
      int pid = map_.addMapPoint(p_world, desc);
      map_.addObservation(pid, prev_kf.id, prev_kp);
      map_.addObservation(pid, new_frame.id, curr_kp);

      map_.keyframes().back().map_point_ids[prev_kp] = pid;
      new_frame.map_point_ids[curr_kp] = pid;
      new_point_positions.push_back(p_world);
      ++new_points;
    }
  }

  int total_tracked = new_frame.numTrackedPoints();
  map_.addKeyframe(std::move(new_frame));

  // Log new points for this frame (yellow highlight)
  if (viz && !new_point_positions.empty()) {
    viz->logNewMapPoints(new_point_positions, frame_idx);
  }

  // ---- Per-frame diagnostic (verbose for first 5 frames, then every 50) ----
  int frames_since_init = frame_idx - map_.keyframes().front().id;
  bool verbose = frames_since_init <= 5 ||
                 map_.numKeyframes() % 50 == 0;
  if (verbose) {
    Eigen::Vector3d pos = T_world_camera.block<3, 1>(0, 3);
    std::cout << "  [F" << std::setw(3) << frame_idx << "] "
              << "matches=" << matches.size()
              << " w3d=" << match_with_3d.size()
              << " wo3d=" << match_without_3d.size()
              << " pnp_in=" << pnp_inliers.rows
              << " new_pts=" << new_points
              << " tracked=" << total_tracked
              << " total_map=" << map_.numPoints()
              << " pos=[" << std::fixed << std::setprecision(3)
              << pos.x() << " " << pos.y() << " " << pos.z() << "]"
              << std::endl;
  }

  return true;
}

// ==========================================================================
// run — main pipeline
// ==========================================================================
void VisualFrontendPipeline::run(const PipelineConfig& config) {
  if (dataset_.rgb_frames.empty()) {
    std::cerr << "No frames loaded." << std::endl;
    return;
  }

  if (config.use_gpu) {
    std::cout << "  Note: Using CPU feature extraction/matching\n"
              << "        (GPU pipeline integration pending)\n";
  }

  std::cout << std::string(60, '=') << std::endl;
  std::cout << "Visual Frontend — Full Sequence Tracking" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "  Sequence:     " << dataset_.base_path << std::endl;
  std::cout << "  Frames:       " << dataset_.rgb_frames.size() << std::endl;
  std::cout << "  Extractor:    " << extractor_->name() << std::endl;
  std::cout << "  Matcher:      " << matcher_->name() << std::endl;
  std::cout << "  Init mode:    " << sensor_config_.initModeName() << std::endl;
  std::cout << std::endl;

  // ---- Rerun visualization ----
  rerun_viz::VFEVisualizer rr_viz;
  bool viz_enabled = config.rerun.enabled() && rr_viz.init(config.rerun);

  // Compute GT alignment: T_world_mocap transforms GT mocap frame to our Z-up world.
  // Our first camera is at T_world_optical in the world frame.
  // GT first camera is at T_mocap_cam0 in the mocap frame.
  // So T_world_mocap = T_world_cam0 * inv(T_mocap_cam0)
  //                  = T_world_optical * inv(T_mocap_cam0)
  Eigen::Matrix4d T_world_mocap = Eigen::Matrix4d::Identity();
  if (!dataset_.ground_truth.empty() && !dataset_.rgb_frames.empty()) {
    Eigen::Matrix4d T_world_optical = tf_tree_->lookup("world", "optical");
    Eigen::Matrix4d T_mocap_cam0 = findNearestGTPose(
        dataset_.ground_truth, dataset_.rgb_frames[0].timestamp);
    T_world_mocap = T_world_optical * T_mocap_cam0.inverse();
  }

  if (viz_enabled && !dataset_.ground_truth.empty()) {
    // Transform GT poses to Z-up world frame before logging
    std::vector<loader::GroundTruthPose> gt_transformed;
    gt_transformed.reserve(dataset_.ground_truth.size());
    for (const auto& pose : dataset_.ground_truth) {
      Eigen::Matrix4d T_mocap_cam = findNearestGTPose(
          dataset_.ground_truth, pose.timestamp);
      Eigen::Matrix4d T_world_cam = T_world_mocap * T_mocap_cam;
      loader::GroundTruthPose tp;
      tp.timestamp = pose.timestamp;
      tp.tx = T_world_cam(0, 3);
      tp.ty = T_world_cam(1, 3);
      tp.tz = T_world_cam(2, 3);
      gt_transformed.push_back(tp);
    }
    rr_viz.logGroundTruth(gt_transformed);
  }

  auto total_start = std::chrono::high_resolution_clock::now();

  // ---- Phase 1: Sensor-adaptive initialization ----
  std::cout << std::string(60, '-') << std::endl;
  std::cout << "Phase 1: Initialization (" << sensor_config_.initModeName()
            << ")" << std::endl;
  std::cout << std::string(60, '-') << std::endl;

  bool init_ok = false;
  int init_last_frame = 0;

  switch (sensor_config_.bestInitMode()) {
    case SensorConfig::InitMode::DEPTH:
      init_ok = initializeFromDepth(0, viz_enabled ? &rr_viz : nullptr);
      init_last_frame = 0;
      break;

    case SensorConfig::InitMode::IMU:
      // IMU init not yet implemented; fall through to monocular
      std::cout << "  (IMU init not implemented, falling back to monocular)"
                << std::endl;
      [[fallthrough]];

    case SensorConfig::InitMode::MONOCULAR:
      init_ok = initializeMonocular(viz_enabled ? &rr_viz : nullptr);
      if (init_ok) {
        init_last_frame = map_.keyframes().back().id;
      }
      break;
  }

  if (!init_ok) {
    std::cerr << "Initialization failed." << std::endl;
    return;
  }

  if (viz_enabled) {
    rr_viz.setFrame(init_last_frame);
    rr_viz.logKeyframes(map_.keyframes());
    rr_viz.logMapPoints(map_.points());
  }

  // ---- Phase 2: Track all subsequent frames ----
  std::cout << std::string(60, '-') << std::endl;
  std::cout << "Phase 2: PnP tracking" << std::endl;
  std::cout << std::string(60, '-') << std::endl;

  int total_frames = static_cast<int>(dataset_.rgb_frames.size());
  int tracked_count = 0;
  int lost_count = 0;
  constexpr int kMaxConsecutiveFailures = 10;

  for (int i = init_last_frame + 1; i < total_frames; ++i) {
    bool ok = trackFrame(i, viz_enabled ? &rr_viz : nullptr);

    if (ok) {
      ++tracked_count;
      tracking_failures_ = 0;

      // Incremental Rerun viz every frame
      if (viz_enabled) {
        rr_viz.setFrame(i);
        rr_viz.logKeyframes(map_.keyframes());
        rr_viz.logMapPoints(map_.points());
      }

      // Periodic console progress
      if (tracked_count % 50 == 0 || i == total_frames - 1) {
        const auto& last_kf = map_.keyframes().back();
        Eigen::Vector3d pos = last_kf.T_world_camera.block<3, 1>(0, 3);
        std::cout << "  Frame " << std::setw(4) << i << "/" << total_frames
                  << " | kf=" << map_.numKeyframes()
                  << " pts=" << map_.numPoints()
                  << " trk=" << last_kf.numTrackedPoints() << " pos=["
                  << std::fixed << std::setprecision(3) << pos.x() << " "
                  << pos.y() << " " << pos.z() << "]" << std::endl;
      }
    } else {
      ++lost_count;
      ++tracking_failures_;
      if (tracking_failures_ >= kMaxConsecutiveFailures) {
        std::cerr << "  Tracking lost at frame " << i
                  << " after " << kMaxConsecutiveFailures
                  << " consecutive failures." << std::endl;
        break;
      }
    }
  }

  auto total_end = std::chrono::high_resolution_clock::now();
  double total_ms =
      std::chrono::duration<double, std::milli>(total_end - total_start)
          .count();

  // ---- Summary ----
  std::cout << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "Tracking complete" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "  Keyframes:       " << map_.numKeyframes() << std::endl;
  std::cout << "  Map points:      " << map_.numPoints() << std::endl;
  std::cout << "  Tracked frames:  " << tracked_count << std::endl;
  std::cout << "  Lost frames:     " << lost_count << std::endl;
  std::cout << "  Total time:      " << std::fixed << std::setprecision(1)
            << total_ms << " ms (" << std::setprecision(1)
            << total_ms / (tracked_count + 2) << " ms/frame)" << std::endl;

  // ---- Trajectory evaluation against ground truth ----
  if (!dataset_.ground_truth.empty()) {
    std::cout << std::endl;
    std::cout << "  Trajectory evaluation (vs ground truth):" << std::endl;

    double sum_sq_err = 0.0;
    int n_eval = 0;
    double max_err = 0.0;

    for (const auto& kf : map_.keyframes()) {
      Eigen::Matrix4d T_mocap_cam =
          findNearestGTPose(dataset_.ground_truth, kf.timestamp);
      Eigen::Matrix4d T_world_cam_gt = T_world_mocap * T_mocap_cam;
      Eigen::Vector3d pos_est = kf.T_world_camera.block<3, 1>(0, 3);
      Eigen::Vector3d pos_gt = T_world_cam_gt.block<3, 1>(0, 3);
      double err = (pos_est - pos_gt).norm();
      sum_sq_err += err * err;
      if (err > max_err) max_err = err;
      ++n_eval;
    }

    if (n_eval > 0) {
      double ate_rmse = std::sqrt(sum_sq_err / n_eval);
      std::cout << "    ATE RMSE:      " << std::fixed << std::setprecision(4)
                << ate_rmse << " m" << std::endl;
      std::cout << "    ATE max:       " << std::fixed << std::setprecision(4)
                << max_err << " m" << std::endl;
      std::cout << "    Eval poses:    " << n_eval << std::endl;
    }
  }
}

}  // namespace substral
