#pragma once

// =============================================================================
// FactorGraphMemoryMap — contiguous storage for all graph entity data.
//
// All entity types (cameras, points, observations, poses, edges) store their
// numeric data in flat vectors owned by a single FactorGraphMemoryMap.
// Individual entity objects hold a shared_ptr to this map and index into it.
//
// This layout enables efficient GPU upload: each vector can be memcpy'd to
// device memory in one shot.
// =============================================================================

#include <map>
#include <vector>

namespace substral {
namespace backend {

// --- Bundle Adjustment strides ---
static constexpr int CAMERA_STRIDE = 9;
static constexpr int POINT_STRIDE = 3;
static constexpr int OBSERVATION_STRIDE = 2;

// --- Pose-graph strides ---
//   Pose: 7 doubles = (x, y, z, qx, qy, qz, qw) — position + quaternion
//   PoseEdge: 7 + 36 = 43 doubles = measurement (7) + full 6x6 info matrix (36)
static constexpr int POSE_STRIDE = 7;
static constexpr int POSE_EDGE_MEASUREMENT_STRIDE = 7;
static constexpr int POSE_EDGE_INFO_STRIDE = 36;
static constexpr int POSE_EDGE_STRIDE =
    POSE_EDGE_MEASUREMENT_STRIDE + POSE_EDGE_INFO_STRIDE;

// --- Landmark strides ---
static constexpr int LANDMARK_STRIDE = 3;

// Pose-landmark edge strides:
//   Measurement stride depends on the sensor model:
//     - Point observation: 3 (x, y, z in sensor frame)
//     - Bearing-only: 2 (azimuth, elevation)
//     - Range-bearing: 3 (range, azimuth, elevation)
//   Info matrix: dim x dim where dim = measurement dimension
//
// We use a flexible stride: measurement (max 3) + info (max 9) = max 12
// The actual dimensions are set per-edge via the measurement_dim field.
static constexpr int POSE_LANDMARK_EDGE_MAX_MEAS_DIM = 3;
static constexpr int POSE_LANDMARK_EDGE_MAX_INFO_DIM = 9;
static constexpr int POSE_LANDMARK_EDGE_STRIDE =
    POSE_LANDMARK_EDGE_MAX_MEAS_DIM + POSE_LANDMARK_EDGE_MAX_INFO_DIM;

struct FactorGraphMemoryMap {
  // --- Bundle Adjustment data ---
  std::vector<double> observers;
  std::vector<double> scene_points;
  std::vector<double> observations;

  std::vector<int> observation_camera_indices;
  std::vector<int> observation_point_indices;

  std::map<int, int> observer_indices;
  std::map<int, int> scene_point_indices;
  std::map<int, int> observation_indices;

  // --- Pose-graph data ---
  // Contiguous storage for SE(3) poses: 7 doubles each (x,y,z,qx,qy,qz,qw)
  std::vector<double> poses;
  // Contiguous storage for pose-pose edges: 43 doubles each
  //   [0..6]: measurement (x,y,z,qx,qy,qz,qw)
  //   [7..42]: 6x6 information matrix in row-major order
  std::vector<double> pose_edges;

  // Edge connectivity: pairs of (from_id, to_id) for each edge
  std::vector<int> pose_edge_from_indices;
  std::vector<int> pose_edge_to_indices;

  // ID -> offset maps
  std::map<int, int> pose_indices;
  std::map<int, int> pose_edge_indices;
};

}  // namespace backend
}  // namespace substral
