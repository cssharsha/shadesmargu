#pragma once

// =============================================================================
// Pipeline — Strategy pattern for different optimization problem types.
//
// Each pipeline encapsulates:
//   - Problem data (BAProblem, PoseGraphProblem, etc.)
//   - Loading from a file
//   - Printing problem statistics
//   - Visualization (Rerun, OpenCV)
//   - Running the solver
//   - Printing results
//
// The Subastral class holds a unique_ptr<Pipeline> and delegates to it.
// New problem types (e.g., visual-inertial SLAM) can be added by
// implementing a new Pipeline subclass.
// =============================================================================

#include <string>

#include "subastral/backend/solver/lm_solver.hpp"
#include "viz/rerun/ba_visualizer.hpp"  // for VizConfig

namespace substral {

/// Configuration passed to every pipeline.
struct PipelineConfig {
  backend::solver::LMConfig solver;
  rerun_viz::VizConfig rerun;
  std::string viz_output_path;  // OpenCV PNG output (BA only)
  bool use_gpu = true;
};

/// Abstract pipeline — one per problem type.
class Pipeline {
 public:
  virtual ~Pipeline() = default;

  /// Load problem data from a file. Returns false on failure.
  virtual bool load(const std::string& filename) = 0;

  /// Run the full pipeline: stats → viz initial → solve → results → viz final.
  virtual void run(const PipelineConfig& config) = 0;

  /// Human-readable name for this pipeline (e.g. "Bundle Adjustment").
  virtual const char* name() const = 0;
};

}  // namespace substral
