#pragma once

// =============================================================================
// BAVisualizer — Rerun-based visualization for Bundle Adjustment
//
// Logs 3D point clouds, camera frustums, and per-iteration solver metrics
// to a Rerun recording. Supports both saving to .rrd files and streaming
// to a live Rerun viewer via gRPC.
//
// Entity hierarchy:
//   /world/points          — 3D point cloud (colored by reprojection error)
//   /world/cameras/cam_N   — Camera frustums with coordinate axes
//   /solver/cost           — Total cost over iterations (time series)
//   /solver/rms            — RMS reprojection error over iterations
//   /solver/lambda         — LM damping parameter over iterations
// =============================================================================

#include <string>
#include <vector>

#include "subastral/backend/common.h"

namespace substral {
namespace rerun_viz {

/// Configuration for the visualizer.
struct VizConfig {
  /// Path to save .rrd file. Empty = don't save.
  std::string rrd_path;

  /// gRPC address to connect to a live Rerun viewer. Empty = don't connect.
  /// Format: "rerun+http://addr:port/proxy" (default:
  /// rerun+http://127.0.0.1:9876/proxy)
  std::string connect_addr;

  /// Whether visualization is enabled at all.
  bool enabled() const { return !rrd_path.empty() || !connect_addr.empty(); }
};

/// Bundle Adjustment visualizer using Rerun.
class BAVisualizer {
 public:
  BAVisualizer() = default;
  ~BAVisualizer();

  /// Initialize the recording stream. Returns false on failure.
  bool init(const VizConfig& config);

  /// Log the full 3D point cloud. Call before/after optimization.
  /// \param label  Entity path suffix, e.g. "initial" or "optimized"
  /// \param points  Flat array of [x,y,z, x,y,z, ...] (num_points * 3)
  /// \param num_points  Number of 3D points
  void logPoints(const std::string& label, const double* points,
                 int num_points);

  /// Log all camera poses as frustums.
  /// \param label  Entity path suffix, e.g. "initial" or "optimized"
  /// \param cameras  Flat array of 9-param cameras [w,t,f,k1,k2, ...]
  /// \param num_cameras  Number of cameras
  void logCameras(const std::string& label, const double* cameras,
                  int num_cameras);

  /// Log a single solver iteration's metrics.
  /// \param iteration  LM iteration number (used as timeline tick)
  /// \param cost       Total cost (sum of squared residuals)
  /// \param rms        RMS reprojection error in pixels
  /// \param lambda     Current LM damping parameter
  void logIteration(int iteration, double cost, double rms, double lambda);

  /// Log the full problem state at a given iteration (points + cameras).
  /// Useful for animating the optimization over time.
  void logState(int iteration, const double* cameras, int num_cameras,
                const double* points, int num_points);

 private:
  struct Impl;
  Impl* impl_ = nullptr;
};

}  // namespace rerun_viz
}  // namespace substral
