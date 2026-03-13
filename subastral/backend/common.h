#pragma once

// =============================================================================
// Umbrella header — includes all graph entity types.
//
// New code should prefer including the specific header it needs:
//   subastral/backend/graph/ba_types.h          — Camera, Point, Observation,
//   BAProblem subastral/backend/graph/pose_graph_types.h  — Pose, PoseEdge,
//   PoseGraphProblem subastral/backend/graph/landmark_types.h    — Landmark,
//   PoseLandmarkEdge subastral/backend/graph/graph_entity.h      — GraphEntity
//   base class subastral/backend/graph/memory_map.h        —
//   FactorGraphMemoryMap + strides
// =============================================================================

#include "subastral/backend/graph/ba_types.h"
#include "subastral/backend/graph/landmark_types.h"
#include "subastral/backend/graph/pose_graph_types.h"
