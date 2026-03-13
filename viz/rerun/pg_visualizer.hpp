#pragma once

// =============================================================================
// PGVisualizer — Rerun-based visualization for Pose-Graph SLAM
//
// Logs 3D pose positions, edges (odometry + loop closures), and per-iteration
// solver metrics to a Rerun recording.
//
// Entity hierarchy:
//   /world/poses/{label}                  — Pose positions as 3D points
//   /world/edges/{label}/odometry         — Odometry edges (individual lines)
//   /world/edges/{label}/loop_closures    — Loop closure edges (individual
//   lines) /world/trajectory/{label}             — Trajectory line strip
//   /solver/cost                          — Total cost time series
//   /solver/lambda                        — LM damping time series
//
// Edge coloring:
//   Each edge is a separate line segment, colored by its residual error norm.
//   Odometry edges use a blue-to-white color ramp (low→high error).
//   Loop closure edges use a green-to-red color ramp (low→high error).
//   The error value (||e||) is available per-edge for inspection.
// =============================================================================

#include <string>
#include <vector>

#include "subastral/backend/common.h"
#include "viz/rerun/ba_visualizer.hpp"  // for VizConfig

namespace substral {
namespace rerun_viz {

/// Pose-Graph SLAM visualizer using Rerun.
class PGVisualizer {
 public:
  PGVisualizer() = default;
  ~PGVisualizer();

  /// Initialize the recording stream. Returns false on failure.
  bool init(const VizConfig& config);

  /// Log pose positions as 3D points.
  /// \param label  "initial" or "optimized"
  /// \param problem  The pose-graph problem (reads pose positions)
  void logPoses(const std::string& label,
                const backend::PoseGraphProblem& problem);

  /// Log edges as individual line segments, colored by per-edge error.
  /// Odometry edges: blue→white ramp by error magnitude.
  /// Loop closure edges: green→red ramp by error magnitude.
  /// \param label  "initial" or "optimized"
  /// \param problem  The pose-graph problem
  void logEdges(const std::string& label,
                const backend::PoseGraphProblem& problem);

  /// Log the trajectory as a connected line strip through poses in ID order.
  /// \param label  "initial" or "optimized"
  /// \param problem  The pose-graph problem
  void logTrajectory(const std::string& label,
                     const backend::PoseGraphProblem& problem);

  /// Log a single solver iteration's metrics.
  void logIteration(int iteration, double cost, double lambda);

  /// Log the full problem state at a given iteration.
  /// Logs poses, edges, trajectory, and solver metrics.
  void logState(int iteration, const backend::PoseGraphProblem& problem,
                double cost, double lambda);

 private:
  struct Impl;
  Impl* impl_ = nullptr;
};

}  // namespace rerun_viz
}  // namespace substral
