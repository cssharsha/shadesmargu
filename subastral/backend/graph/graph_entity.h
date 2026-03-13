#pragma once

// =============================================================================
// GraphEntity — abstract base class for all factor graph entities.
//
// Every entity (Camera, Point, Observation, Pose, PoseEdge, Landmark, etc.)
// derives from GraphEntity and stores its numeric data in a shared
// FactorGraphMemoryMap.
// =============================================================================

#include <memory>

#include "subastral/backend/graph/memory_map.h"

namespace substral {
namespace backend {

class GraphEntity {
 public:
  enum Type { OBSERVER, SCENE_POINT, OBSERVATION, POSE, POSE_EDGE, NONE };
  GraphEntity(Type type, int id, int stride,
              std::shared_ptr<FactorGraphMemoryMap> memory_map)
      : type(type), id(id), stride(stride), memory_map(memory_map) {}

  Type get_type() const { return type; }

  int get_id() const { return id; }

  int get_stride() const { return stride; }

  virtual double* mutable_data() = 0;

  virtual const double* data() const = 0;

  virtual ~GraphEntity() = default;

 protected:
  Type type = NONE;
  int id = -1;
  int stride = 2;

  std::shared_ptr<FactorGraphMemoryMap> memory_map;
};

}  // namespace backend
}  // namespace substral
