#include "subastral/pose_graph_pipeline.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "subastral/backend/solver/lm_solver.hpp"
#include "subastral/backend/solver/pose_graph_solver.cuh"
#include "subastral/loader/g2o_loader.h"
#include "viz/rerun/pg_visualizer.hpp"

namespace substral {

// =============================================================================
// Shared helper (also in ba_pipeline.cpp — small duplication is fine for
// decoupled compilation units)
// =============================================================================

static void printResult(const backend::solver::LMResult& result,
                        double elapsed_ms) {
  std::cout << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "Optimization complete" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "  Converged:      " << (result.converged ? "yes" : "NO")
            << std::endl;
  std::cout << "  Reason:         " << result.termination_reason << std::endl;
  std::cout << "  Iterations:     " << result.iterations << std::endl;
  std::cout << "  Initial cost:   " << result.initial_cost << std::endl;
  std::cout << "  Final cost:     " << result.final_cost << std::endl;
  std::cout << "  Cost reduction: " << std::fixed << std::setprecision(2)
            << (1.0 - result.final_cost / (result.initial_cost + 1e-30)) * 100.0
            << "%" << std::defaultfloat << std::endl;
  std::cout << "  Wall time:      " << std::fixed << std::setprecision(1)
            << elapsed_ms << " ms" << std::defaultfloat << std::endl;
}

// =============================================================================
// PoseGraphPipeline
// =============================================================================

bool PoseGraphPipeline::load(const std::string& filename) {
  loader::G2OLoader loader(filename);
  return loader.Load(problem_);
}

void PoseGraphPipeline::run(const PipelineConfig& config) {
  if (problem_.poses.empty()) {
    std::cerr << "No poses loaded. Nothing to optimize." << std::endl;
    return;
  }

  std::cout << std::string(60, '=') << std::endl;
  std::cout << "Subastral Pose-Graph SLAM (GPU)" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "  Poses:          " << problem_.get_num_poses() << std::endl;
  std::cout << "  Edges:          " << problem_.get_num_edges() << std::endl;
  std::cout << "  Fixed vertex:   " << problem_.fixed_vertex_id << std::endl;
  std::cout << "  Max iterations: " << config.solver.max_iterations
            << std::endl;
  std::cout << "  Initial lambda: " << config.solver.initial_lambda
            << std::endl;
  std::cout << std::endl;

  // ---- Rerun visualization: log initial state ----
  rerun_viz::PGVisualizer rr_viz;
  if (config.rerun.enabled() && rr_viz.init(config.rerun)) {
    rr_viz.logPoses("initial", problem_);
    rr_viz.logEdges("initial", problem_);
    rr_viz.logTrajectory("initial", problem_);
  }

  // ---- Run solver ----
  std::cout << std::string(60, '-') << std::endl;
  std::cout << "Optimizing..." << std::endl;
  std::cout << std::string(60, '-') << std::endl;

  auto start = std::chrono::high_resolution_clock::now();

  // Set up callback for Rerun per-iteration logging
  backend::solver::PoseGraphCallback callback = nullptr;
  if (config.rerun.enabled()) {
    callback = [&](int iter, double cost) -> bool {
      rr_viz.logIteration(iter, cost, config.solver.initial_lambda);
      return true;
    };
  }

  backend::solver::LMResult result =
      backend::solver::solvePoseGraph_GPU(problem_, config.solver, callback);

  auto end = std::chrono::high_resolution_clock::now();
  double elapsed_ms =
      std::chrono::duration<double, std::milli>(end - start).count();

  // ---- Results ----
  printResult(result, elapsed_ms);

  // ---- Rerun visualization: log final state ----
  if (config.rerun.enabled()) {
    rr_viz.logPoses("optimized", problem_);
    rr_viz.logEdges("optimized", problem_);
    rr_viz.logTrajectory("optimized", problem_);
  }
}

}  // namespace substral
