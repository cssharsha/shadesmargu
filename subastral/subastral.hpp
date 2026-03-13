#pragma once

#include <memory>
#include <string>

#include "subastral/pipeline.hpp"

namespace substral {

// =============================================================================
// Subastral — top-level entry point.
//
// Holds a Pipeline (strategy) that encapsulates the problem type, loader,
// solver, and visualizer. The pipeline is created based on the mode
// (BA vs pose-graph) and then run() is called.
//
// Usage:
//   Subastral app;
//   app.createPipeline("pose-graph");   // or "ba" (default)
//   app.config().solver.max_iterations = 30;
//   app.load("dataset.g2o");
//   app.run();
// =============================================================================

class Subastral {
 public:
  Subastral();

  /// Create the pipeline for the given mode.
  /// \param mode  "ba" (default) or "pose-graph"
  void createPipeline(const std::string& mode);

  /// Load a dataset file. The pipeline must be created first.
  bool load(const std::string& filename);

  /// Run the full optimization pipeline.
  void run();

  /// Access the pipeline config for customization before calling run().
  PipelineConfig& config() { return config_; }
  const PipelineConfig& config() const { return config_; }

 private:
  std::unique_ptr<Pipeline> pipeline_;
  PipelineConfig config_;
};

}  // namespace substral
