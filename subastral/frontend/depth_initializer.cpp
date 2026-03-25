#include "subastral/frontend/depth_initializer.hpp"

#include <cmath>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

namespace substral {
namespace frontend {

int DepthInitializer::initialize(Frame& frame,
                                 const std::string& depth_image_path,
                                 const CameraIntrinsics& intrinsics, Map& map,
                                 double depth_scale, double min_depth,
                                 double max_depth) {
  cv::Mat depth_raw = cv::imread(depth_image_path, cv::IMREAD_UNCHANGED);
  if (depth_raw.empty()) {
    std::cerr << "DepthInitializer: failed to load " << depth_image_path
              << std::endl;
    return 0;
  }
  if (depth_raw.type() != CV_16UC1) {
    std::cerr << "DepthInitializer: expected CV_16UC1, got type="
              << depth_raw.type() << std::endl;
    return 0;
  }

  // T_world_camera must be set by the caller (defines world convention)
  frame.map_point_ids.resize(frame.keypoints.size(), -1);

  Eigen::Matrix3d R_wc = frame.T_world_camera.block<3, 3>(0, 0);
  Eigen::Vector3d t_wc = frame.T_world_camera.block<3, 1>(0, 3);

  int valid_count = 0;
  for (int i = 0; i < static_cast<int>(frame.keypoints.size()); ++i) {
    const cv::KeyPoint& kp = frame.keypoints[i];

    int u = static_cast<int>(std::round(kp.pt.x));
    int v = static_cast<int>(std::round(kp.pt.y));

    if (u < 0 || u >= depth_raw.cols || v < 0 || v >= depth_raw.rows) continue;

    uint16_t raw = depth_raw.at<uint16_t>(v, u);
    if (raw == 0) continue;

    double z = static_cast<double>(raw) / depth_scale;
    if (z < min_depth || z > max_depth) continue;

    // Backproject to 3D camera coordinates, then transform to world
    double x = (kp.pt.x - intrinsics.cx) / intrinsics.fx * z;
    double y = (kp.pt.y - intrinsics.cy) / intrinsics.fy * z;
    Eigen::Vector3d p_cam(x, y, z);
    Eigen::Vector3d p_world = R_wc * p_cam + t_wc;

    cv::Mat desc = frame.descriptors.row(i);
    int pid = map.addMapPoint(p_world, desc);
    map.addObservation(pid, frame.id, i);
    frame.map_point_ids[i] = pid;
    ++valid_count;
  }

  return valid_count;
}

}  // namespace frontend
}  // namespace substral
