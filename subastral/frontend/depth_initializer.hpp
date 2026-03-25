#pragma once

#include <Eigen/Core>
#include <string>

#include "subastral/types/camera.h"
#include "subastral/types/frame.h"
#include "subastral/types/map.h"

namespace substral {
namespace frontend {

/// Depth-based map initialization from a single RGB-D frame.
///
/// For each extracted keypoint, looks up the corresponding depth value
/// in the aligned depth image and backprojects to 3D using the camera
/// intrinsics. This yields metric-scale 3D points without triangulation.
///
/// Depth images are 16-bit PNG; divide raw value by depth_scale to get meters
/// (TUM RGB-D convention: depth_scale = 5000).
/// Pixels with depth == 0 are invalid (no reading).
class DepthInitializer {
 public:
  /// Initialize map from a single frame + its aligned depth image.
  ///
  /// @param frame  Frame with extracted keypoints and descriptors.
  ///               T_world_camera is set to identity (first frame = world origin).
  /// @param depth_image_path  Absolute path to 16-bit depth PNG.
  /// @param intrinsics  Camera intrinsics (fx, fy, cx, cy).
  /// @param map  Map to populate with backprojected 3D points.
  /// @param depth_scale  Divisor to convert raw 16-bit to meters (default: 5000.0).
  /// @param min_depth  Minimum valid depth in meters (default: 0.1).
  /// @param max_depth  Maximum valid depth in meters (default: 10.0).
  /// @return Number of valid 3D points created, or 0 on failure.
  int initialize(Frame& frame, const std::string& depth_image_path,
                 const CameraIntrinsics& intrinsics, Map& map,
                 double depth_scale = 5000.0, double min_depth = 0.1,
                 double max_depth = 10.0);
};

}  // namespace frontend
}  // namespace substral
