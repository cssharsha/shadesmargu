#pragma once

// =============================================================================
// Bundle Adjustment types — Camera, Point, Observation, BAProblem.
// =============================================================================

#include <memory>
#include <stdexcept>
#include <vector>

#include "subastral/backend/graph/graph_entity.h"

namespace substral {
namespace backend {

class Camera : public GraphEntity {
 public:
  Camera(int id, std::shared_ptr<FactorGraphMemoryMap> memory_map)
      : GraphEntity(OBSERVER, id, CAMERA_STRIDE, memory_map) {
    memory_map->observers.resize(memory_map->observers.size() + 9);
    memory_map->observer_indices[id] = memory_map->observers.size() - 9;
  }

  double* mutable_data() {
    return &memory_map->observers[memory_map->observer_indices[id]];
  }

  const double* data() const {
    return &memory_map->observers[memory_map->observer_indices[id]];
  }
};

class Point : public GraphEntity {
 public:
  Point(int id, std::shared_ptr<FactorGraphMemoryMap> memory_map)
      : GraphEntity{SCENE_POINT, id, POINT_STRIDE, memory_map} {
    memory_map->scene_points.resize(memory_map->scene_points.size() + 3);
    memory_map->scene_point_indices[id] = memory_map->scene_points.size() - 3;
  }

  double* mutable_data() override {
    return &memory_map->scene_points[memory_map->scene_point_indices[id]];
  }

  const double* data() const override {
    return &memory_map->scene_points[memory_map->scene_point_indices[id]];
  }
};

class Observation : public GraphEntity {
 public:
  Observation(int id, int camera_id, int point_id,
              std::shared_ptr<FactorGraphMemoryMap> memory_map)
      : GraphEntity(OBSERVATION, id, OBSERVATION_STRIDE, memory_map),
        camera_id(camera_id),
        point_id(point_id) {
    memory_map->observations.resize(memory_map->observations.size() + 2);
    memory_map->observation_indices[id] = memory_map->observations.size() - 2;
    memory_map->observation_camera_indices.push_back(camera_id);
    memory_map->observation_point_indices.push_back(point_id);
  }

  double* mutable_data() override {
    throw std::runtime_error("Observations are not mutable");
  }

  const double* data() const override {
    return &memory_map->observations[memory_map->observation_indices[id]];
  }

  int get_camera_id() const { return camera_id; }

  int get_point_id() const { return point_id; }

 private:
  int camera_id;
  int point_id;
};

struct BAProblem {
  std::vector<std::shared_ptr<Camera>> cameras;
  std::vector<std::shared_ptr<Point>> points;
  std::vector<std::shared_ptr<Observation>> observations;
  std::shared_ptr<FactorGraphMemoryMap> memory_map;

  BAProblem() : memory_map(std::make_shared<FactorGraphMemoryMap>()) {}

  double* mutable_camera_data() { return memory_map->observers.data(); }

  double* mutable_point_data() { return memory_map->scene_points.data(); }

  double* mutable_observation_data() { return memory_map->observations.data(); }
  int get_num_observations() { return observations.size(); }
  int* mutable_observation_camera_indices() {
    return memory_map->observation_camera_indices.data();
  }
  int* mutable_observation_point_indices() {
    return memory_map->observation_point_indices.data();
  }
};

}  // namespace backend
}  // namespace substral
