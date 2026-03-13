#pragma once

// =============================================================================
// Pose-Graph SLAM types — Pose, PoseEdge, PoseGraphProblem.
//
// SE(3) poses stored as 7 doubles: (x, y, z, qx, qy, qz, qw)
//   - (x, y, z): translation
//   - (qx, qy, qz, qw): unit quaternion (Hamilton convention, w last)
//
// Pose-pose edges stored as 43 doubles:
//   - [0..6]:  relative measurement (x, y, z, qx, qy, qz, qw)
//   - [7..42]: 6x6 information matrix, row-major
//              Ordering: (w0, w1, w2, v0, v1, v2) — rotation first
// =============================================================================

#include <cmath>
#include <map>
#include <memory>
#include <vector>

#include "subastral/backend/graph/graph_entity.h"

namespace substral {
namespace backend {

class Pose : public GraphEntity {
 public:
  Pose(int id, std::shared_ptr<FactorGraphMemoryMap> memory_map)
      : GraphEntity(POSE, id, POSE_STRIDE, memory_map) {
    memory_map->poses.resize(memory_map->poses.size() + POSE_STRIDE);
    memory_map->pose_indices[id] =
        static_cast<int>(memory_map->poses.size()) - POSE_STRIDE;
    // Initialize to identity: (0,0,0, 0,0,0,1)
    double* d = mutable_data();
    d[0] = d[1] = d[2] = 0.0;  // translation
    d[3] = d[4] = d[5] = 0.0;  // qx, qy, qz
    d[6] = 1.0;                // qw
  }

  double* mutable_data() override {
    return &memory_map->poses[memory_map->pose_indices[id]];
  }

  const double* data() const override {
    return &memory_map->poses[memory_map->pose_indices[id]];
  }

  // Convenience: translation as (x, y, z)
  double x() const { return data()[0]; }
  double y() const { return data()[1]; }
  double z() const { return data()[2]; }

  // Convenience: quaternion as (qx, qy, qz, qw)
  double qx() const { return data()[3]; }
  double qy() const { return data()[4]; }
  double qz() const { return data()[5]; }
  double qw() const { return data()[6]; }
};

class PoseEdge : public GraphEntity {
 public:
  PoseEdge(int edge_id, int from_id, int to_id,
           std::shared_ptr<FactorGraphMemoryMap> memory_map)
      : GraphEntity(POSE_EDGE, edge_id, POSE_EDGE_STRIDE, memory_map),
        from_id(from_id),
        to_id(to_id) {
    memory_map->pose_edges.resize(memory_map->pose_edges.size() +
                                  POSE_EDGE_STRIDE);
    memory_map->pose_edge_indices[edge_id] =
        static_cast<int>(memory_map->pose_edges.size()) - POSE_EDGE_STRIDE;
    memory_map->pose_edge_from_indices.push_back(from_id);
    memory_map->pose_edge_to_indices.push_back(to_id);

    // Initialize measurement to identity
    double* m = mutable_measurement_data();
    m[0] = m[1] = m[2] = 0.0;
    m[3] = m[4] = m[5] = 0.0;
    m[6] = 1.0;

    // Initialize information matrix to identity (row-major 6x6)
    double* info = mutable_info_data();
    for (int i = 0; i < 36; ++i) info[i] = 0.0;
    for (int i = 0; i < 6; ++i) info[i * 6 + i] = 1.0;
  }

  double* mutable_data() override {
    return &memory_map->pose_edges[memory_map->pose_edge_indices[id]];
  }

  const double* data() const override {
    return &memory_map->pose_edges[memory_map->pose_edge_indices[id]];
  }

  // Measurement: first 7 doubles
  double* mutable_measurement_data() { return mutable_data(); }
  const double* measurement_data() const { return data(); }

  // Information matrix: next 36 doubles (row-major 6x6)
  double* mutable_info_data() {
    return mutable_data() + POSE_EDGE_MEASUREMENT_STRIDE;
  }
  const double* info_data() const {
    return data() + POSE_EDGE_MEASUREMENT_STRIDE;
  }

  int get_from_id() const { return from_id; }
  int get_to_id() const { return to_id; }

  // Returns true if this is a loop closure (non-sequential edge)
  bool is_loop_closure(int threshold = 1) const {
    return std::abs(to_id - from_id) > threshold;
  }

 private:
  int from_id;
  int to_id;
};

struct PoseGraphProblem {
  // Pose-pose edges (odometry + loop closures)
  std::vector<std::shared_ptr<Pose>> poses;
  std::vector<std::shared_ptr<PoseEdge>> edges;

  // Landmarks and pose-landmark edges are in landmark_types.h;
  // forward-declared here for the optional fields.
  // (Included via common.h umbrella when needed.)
  std::shared_ptr<FactorGraphMemoryMap> memory_map;

  // The ID of the fixed vertex (gauge freedom — first pose is anchored)
  int fixed_vertex_id = -1;

  PoseGraphProblem() : memory_map(std::make_shared<FactorGraphMemoryMap>()) {}

  double* mutable_pose_data() { return memory_map->poses.data(); }
  const double* pose_data() const { return memory_map->poses.data(); }

  double* mutable_edge_data() { return memory_map->pose_edges.data(); }
  const double* edge_data() const { return memory_map->pose_edges.data(); }

  int* mutable_edge_from_indices() {
    return memory_map->pose_edge_from_indices.data();
  }
  int* mutable_edge_to_indices() {
    return memory_map->pose_edge_to_indices.data();
  }

  int get_num_poses() const { return static_cast<int>(poses.size()); }
  int get_num_edges() const { return static_cast<int>(edges.size()); }

  int get_num_loop_closures(int threshold = 1) const {
    int count = 0;
    for (const auto& e : edges) {
      if (e->is_loop_closure(threshold)) ++count;
    }
    return count;
  }

  // Get sorted vertex IDs (std::map is already sorted)
  std::vector<int> sorted_vertex_ids() const {
    std::vector<int> ids;
    ids.reserve(memory_map->pose_indices.size());
    for (const auto& kv : memory_map->pose_indices) {
      ids.push_back(kv.first);
    }
    return ids;
  }

  // Build mapping: vertex ID -> index in [0, N-1], fixed vertex maps to -1
  std::map<int, int> build_vertex_index_map() const {
    std::map<int, int> id_to_idx;
    int idx = 0;
    for (const auto& kv : memory_map->pose_indices) {
      if (kv.first == fixed_vertex_id) {
        id_to_idx[kv.first] = -1;
      } else {
        id_to_idx[kv.first] = idx++;
      }
    }
    return id_to_idx;
  }

  // Total DOF = 6 * (num_poses - 1), one pose fixed for gauge freedom
  int get_num_free_params() const {
    int n = get_num_poses();
    if (fixed_vertex_id >= 0) --n;
    return 6 * n;
  }
};

}  // namespace backend
}  // namespace substral
