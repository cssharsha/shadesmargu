#pragma once

// =============================================================================
// Landmark types for Pose-Graph SLAM with visual observations.
//
// 3D point landmarks stored as 3 doubles: (x, y, z).
// Used when the pose-graph includes pose-to-landmark observations
// (e.g., bearing-only, range, or full 3D point measurements).
//
// When landmarks are present, the Hessian has the bipartite structure:
//
//   H = | H_pp   H_pl |    H_pp = pose-pose blocks (from pose-pose edges)
//       | H_pl^T H_ll |    H_ll = landmark-landmark blocks (block-diagonal)
//                           H_pl = pose-landmark cross terms
//
// This enables Schur complement elimination of landmarks:
//   S = H_pp - H_pl * H_ll^{-1} * H_pl^T
// =============================================================================

#include <memory>

#include "subastral/backend/graph/graph_entity.h"

namespace substral {
namespace backend {

class Landmark : public GraphEntity {
 public:
  enum class Type { POINT_3D };  // Extensible for other landmark types

  Landmark(int id, std::shared_ptr<FactorGraphMemoryMap> memory_map)
      : GraphEntity(SCENE_POINT, id, LANDMARK_STRIDE, memory_map) {
    // Reuse scene_points storage from FactorGraphMemoryMap
    memory_map->scene_points.resize(memory_map->scene_points.size() +
                                    LANDMARK_STRIDE);
    memory_map->scene_point_indices[id] =
        static_cast<int>(memory_map->scene_points.size()) - LANDMARK_STRIDE;
    // Initialize to origin
    double* d = mutable_data();
    d[0] = d[1] = d[2] = 0.0;
  }

  double* mutable_data() override {
    return &memory_map->scene_points[memory_map->scene_point_indices[id]];
  }

  const double* data() const override {
    return &memory_map->scene_points[memory_map->scene_point_indices[id]];
  }

  double x() const { return data()[0]; }
  double y() const { return data()[1]; }
  double z() const { return data()[2]; }
};

/// Pose-to-landmark edge with a configurable measurement model.
///
/// The measurement dimension (and thus the Jacobian/residual size) depends
/// on the sensor model. For a 3D point observation, meas_dim=3 and the
/// residual is e = R_i^T * (landmark - t_i) - z_measured.
/// For bearing-only, meas_dim=2, etc.
///
/// The information matrix is meas_dim x meas_dim, stored row-major.
class PoseLandmarkEdge : public GraphEntity {
 public:
  enum class SensorModel {
    POINT_3D,       // Full 3D observation: z = R^T * (p - t), dim=3
    BEARING_ONLY,   // Bearing: z = (az, el), dim=2
    RANGE_BEARING,  // Range + bearing: z = (r, az, el), dim=3
  };

  PoseLandmarkEdge(int edge_id, int pose_id, int landmark_id, SensorModel model,
                   std::shared_ptr<FactorGraphMemoryMap> memory_map)
      : GraphEntity(POSE_EDGE, edge_id, POSE_LANDMARK_EDGE_STRIDE, memory_map),
        pose_id_(pose_id),
        landmark_id_(landmark_id),
        sensor_model_(model) {
    switch (model) {
      case SensorModel::POINT_3D:
        meas_dim_ = 3;
        break;
      case SensorModel::BEARING_ONLY:
        meas_dim_ = 2;
        break;
      case SensorModel::RANGE_BEARING:
        meas_dim_ = 3;
        break;
    }

    // Allocate storage in pose_edges (reusing the same contiguous buffer)
    memory_map->pose_edges.resize(memory_map->pose_edges.size() +
                                  POSE_LANDMARK_EDGE_STRIDE);
    memory_map->pose_edge_indices[edge_id] =
        static_cast<int>(memory_map->pose_edges.size()) -
        POSE_LANDMARK_EDGE_STRIDE;

    // Zero-initialize
    double* d = mutable_data();
    for (int i = 0; i < POSE_LANDMARK_EDGE_STRIDE; ++i) d[i] = 0.0;

    // Identity info matrix (meas_dim x meas_dim)
    double* info = mutable_info_data();
    for (int i = 0; i < meas_dim_; ++i) info[i * meas_dim_ + i] = 1.0;
  }

  double* mutable_data() override {
    return &memory_map->pose_edges[memory_map->pose_edge_indices[id]];
  }

  const double* data() const override {
    return &memory_map->pose_edges[memory_map->pose_edge_indices[id]];
  }

  double* mutable_measurement_data() { return mutable_data(); }
  const double* measurement_data() const { return data(); }

  double* mutable_info_data() {
    return mutable_data() + POSE_LANDMARK_EDGE_MAX_MEAS_DIM;
  }
  const double* info_data() const {
    return data() + POSE_LANDMARK_EDGE_MAX_MEAS_DIM;
  }

  int get_pose_id() const { return pose_id_; }
  int get_landmark_id() const { return landmark_id_; }
  int get_meas_dim() const { return meas_dim_; }
  SensorModel get_sensor_model() const { return sensor_model_; }

 private:
  int pose_id_;
  int landmark_id_;
  int meas_dim_;
  SensorModel sensor_model_;
};

}  // namespace backend
}  // namespace substral
