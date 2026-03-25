#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <unordered_map>
#include <vector>

#include "subastral/types/frame.h"
#include "subastral/types/map_point.h"

namespace substral {

/// Sparse 3D map: collection of map points and keyframes.
///
/// This is the central data structure for the visual front-end.
/// Map points are triangulated 3D features observed by multiple frames.
/// Keyframes are frames with extracted features and estimated poses.
class Map {
 public:
  /// Add a new map point. Returns its assigned ID.
  int addMapPoint(const Eigen::Vector3d& position, const cv::Mat& descriptor,
                  uint8_t r = 128, uint8_t g = 128, uint8_t b = 128) {
    MapPoint mp;
    mp.id = next_point_id_++;
    mp.position = position;
    mp.descriptor = descriptor.clone();
    mp.r = r;
    mp.g = g;
    mp.b = b;
    points_[mp.id] = std::move(mp);
    return mp.id;
  }

  /// Record that map point `point_id` is observed in `frame_id` at keypoint
  /// index `kp_idx`.
  void addObservation(int point_id, int frame_id, int kp_idx) {
    auto it = points_.find(point_id);
    if (it != points_.end()) {
      it->second.observations.emplace_back(frame_id, kp_idx);
    }
  }

  /// Get a map point by ID. Throws if not found.
  MapPoint& getPoint(int id) { return points_.at(id); }
  const MapPoint& getPoint(int id) const { return points_.at(id); }

  /// Check if a map point exists.
  bool hasPoint(int id) const { return points_.count(id) > 0; }

  /// Remove a map point by ID.
  void removePoint(int id) { points_.erase(id); }

  /// Add a keyframe (frame with features and pose).
  void addKeyframe(Frame frame) { keyframes_.push_back(std::move(frame)); }

  /// Access all keyframes.
  const std::vector<Frame>& keyframes() const { return keyframes_; }
  std::vector<Frame>& keyframes() { return keyframes_; }

  /// Access all map points.
  const std::unordered_map<int, MapPoint>& points() const { return points_; }
  std::unordered_map<int, MapPoint>& points() { return points_; }

  /// Number of map points.
  int numPoints() const { return static_cast<int>(points_.size()); }

  /// Number of keyframes.
  int numKeyframes() const { return static_cast<int>(keyframes_.size()); }

  /// Clear all data.
  void clear() {
    points_.clear();
    keyframes_.clear();
    next_point_id_ = 0;
  }

 private:
  std::unordered_map<int, MapPoint> points_;
  std::vector<Frame> keyframes_;
  int next_point_id_ = 0;
};

}  // namespace substral
