// =============================================================================
// VFEVisualizer implementation — Rerun C++ SDK
// =============================================================================

#include "viz/rerun/vfe_visualizer.hpp"

#include <iostream>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <rerun.hpp>
#include <vector>

namespace substral {
namespace rerun_viz {

// ---------------------------------------------------------------------------
// Pimpl — hides the rerun::RecordingStream from the header
// ---------------------------------------------------------------------------
struct VFEVisualizer::Impl {
  std::unique_ptr<rerun::RecordingStream> rec;
};

VFEVisualizer::~VFEVisualizer() { delete impl_; }

bool VFEVisualizer::init(const VizConfig& config) {
  if (!config.enabled()) return false;

  impl_ = new Impl();
  impl_->rec =
      std::make_unique<rerun::RecordingStream>("subastral_vfe", "default");

  if (!config.rrd_path.empty()) {
    impl_->rec->save(config.rrd_path).exit_on_failure();
    std::cout << "[viz] Saving recording to: " << config.rrd_path << std::endl;
  }

  if (!config.connect_addr.empty()) {
    impl_->rec->connect_grpc(config.connect_addr).exit_on_failure();
    std::cout << "[viz] Connecting to Rerun viewer at: " << config.connect_addr
              << std::endl;
  }

  // TUM ground truth uses a right-hand coordinate system defined by the
  // motion capture system. No single axis is consistently "up" across all
  // sequences, so we don't force a specific convention here.

  return true;
}

void VFEVisualizer::logGroundTruth(
    const std::vector<loader::GroundTruthPose>& gt) {
  if (!impl_) return;

  std::vector<rerun::Position3D> positions;
  positions.reserve(gt.size());
  for (const auto& pose : gt) {
    positions.push_back(rerun::Position3D(static_cast<float>(pose.tx),
                                          static_cast<float>(pose.ty),
                                          static_cast<float>(pose.tz)));
  }

  impl_->rec->log("world/trajectory/ground_truth",
                  rerun::LineStrips3D(rerun::LineStrip3D(positions))
                      .with_colors(rerun::Color(0, 200, 0))  // green
                      .with_radii(rerun::Radius(0.005f)));
}

void VFEVisualizer::logKeyframes(const std::vector<Frame>& keyframes) {
  if (!impl_) return;

  // Log trajectory as line strip
  std::vector<rerun::Position3D> positions;
  positions.reserve(keyframes.size());
  for (const auto& kf : keyframes) {
    float x = static_cast<float>(kf.T_world_camera(0, 3));
    float y = static_cast<float>(kf.T_world_camera(1, 3));
    float z = static_cast<float>(kf.T_world_camera(2, 3));
    positions.push_back(rerun::Position3D(x, y, z));
  }

  if (positions.size() >= 2) {
    impl_->rec->log("world/trajectory/estimated",
                    rerun::LineStrips3D(rerun::LineStrip3D(positions))
                        .with_colors(rerun::Color(0, 100, 255))  // blue
                        .with_radii(rerun::Radius(0.005f)));
  }

  // Log individual camera positions as points
  impl_->rec->log("world/cameras",
                  rerun::Points3D(positions)
                      .with_colors(rerun::Color(255, 100, 0))  // orange
                      .with_radii(rerun::Radius(0.02f)));
}

void VFEVisualizer::logMapPoints(
    const std::unordered_map<int, MapPoint>& points) {
  if (!impl_) return;

  std::vector<rerun::Position3D> positions;
  std::vector<rerun::Color> colors;
  positions.reserve(points.size());
  colors.reserve(points.size());

  for (const auto& [id, mp] : points) {
    positions.push_back(rerun::Position3D(static_cast<float>(mp.position.x()),
                                          static_cast<float>(mp.position.y()),
                                          static_cast<float>(mp.position.z())));

    colors.push_back(rerun::Color(mp.r, mp.g, mp.b));
  }

  impl_->rec->log("world/map_points",
                  rerun::Points3D(positions).with_colors(colors).with_radii(
                      rerun::Radius(0.01f)));
}

void VFEVisualizer::logDepthPoints(const std::vector<Eigen::Vector3d>& points) {
  if (!impl_) return;

  std::vector<rerun::Position3D> positions;
  positions.reserve(points.size());
  for (const auto& p : points) {
    positions.push_back(rerun::Position3D(static_cast<float>(p.x()),
                                          static_cast<float>(p.y()),
                                          static_cast<float>(p.z())));
  }

  impl_->rec->log("world/depth_gt_points",
                  rerun::Points3D(positions)
                      .with_colors(rerun::Color(255, 50, 50))  // red
                      .with_radii(rerun::Radius(0.008f)));
}

void VFEVisualizer::setFrame(int frame_id) {
  if (!impl_) return;
  impl_->rec->set_time_sequence("frame", frame_id);
}

void VFEVisualizer::logNewMapPoints(const std::vector<Eigen::Vector3d>& points,
                                     int frame_id) {
  if (!impl_ || points.empty()) return;

  std::vector<rerun::Position3D> positions;
  positions.reserve(points.size());
  for (const auto& p : points) {
    positions.push_back(rerun::Position3D(static_cast<float>(p.x()),
                                          static_cast<float>(p.y()),
                                          static_cast<float>(p.z())));
  }

  impl_->rec->log("world/new_points",
                  rerun::Points3D(positions)
                      .with_colors(rerun::Color(255, 255, 0))  // yellow
                      .with_radii(rerun::Radius(0.015f)));
}

void VFEVisualizer::logMatches(const std::string& label, const Frame& frame1,
                               const Frame& frame2,
                               const std::vector<cv::DMatch>& matches,
                               const std::vector<int>& inlier_indices,
                               const cv::Mat& img1, const cv::Mat& img2) {
  if (!impl_) return;

  // Create a side-by-side image with match lines
  cv::Mat combined;
  int w = img1.cols;

  if (img1.channels() == 1) {
    cv::Mat img1_color, img2_color;
    cv::cvtColor(img1, img1_color, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img2, img2_color, cv::COLOR_GRAY2BGR);
    cv::hconcat(img1_color, img2_color, combined);
  } else {
    cv::hconcat(img1, img2, combined);
  }

  // Draw inlier matches in green
  for (int idx : inlier_indices) {
    if (idx < 0 || idx >= static_cast<int>(matches.size())) continue;
    const auto& m = matches[idx];
    cv::Point2f pt1 = frame1.keypoints[m.queryIdx].pt;
    cv::Point2f pt2 = frame2.keypoints[m.trainIdx].pt;
    pt2.x += w;  // offset for side-by-side

    cv::line(combined, pt1, pt2, cv::Scalar(0, 200, 0), 1);
    cv::circle(combined, pt1, 3, cv::Scalar(0, 255, 0), -1);
    cv::circle(combined, pt2, 3, cv::Scalar(0, 255, 0), -1);
  }

  // Log as Rerun image
  // Convert BGR to RGB for Rerun
  cv::Mat rgb;
  cv::cvtColor(combined, rgb, cv::COLOR_BGR2RGB);

  impl_->rec->log(
      "images/matches_" + label,
      rerun::Image::from_rgb24(
          rerun::Collection<uint8_t>::borrow(rgb.data,
                                             rgb.total() * rgb.elemSize()),
          {static_cast<uint32_t>(rgb.cols), static_cast<uint32_t>(rgb.rows)}));
}

}  // namespace rerun_viz
}  // namespace substral
