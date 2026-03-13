#include "subastral/ba_pipeline.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "subastral/backend/ops/errors.h"
#include "subastral/backend/solver/lm_solver.hpp"
#include "subastral/backend/solver/lm_solver_gpu.cuh"
#include "subastral/loader/bal_loader.h"
#include "subastral/viz/ba_visualizer.hpp"
#include "viz/rerun/ba_visualizer.hpp"

namespace substral {

// =============================================================================
// Shared helper
// =============================================================================

static const char* lossTypeStr(backend::solver::LossType type) {
  switch (type) {
    case backend::solver::LossType::HUBER:
      return "Huber";
    case backend::solver::LossType::CAUCHY:
      return "Cauchy";
    case backend::solver::LossType::TRIVIAL:
    default:
      return "L2 (Trivial)";
  }
}

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
// BAPipeline
// =============================================================================

bool BAPipeline::load(const std::string& filename) {
  loader::BALLoader loader(filename);
  return loader.Load(problem_);
}

void BAPipeline::run(const PipelineConfig& config) {
  if (problem_.observations.empty()) {
    std::cerr << "No observations loaded. Nothing to optimize." << std::endl;
    return;
  }

  // ---- Problem summary ----
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "Subastral Bundle Adjustment" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  printProblemStats();
  std::cout << std::endl;

  // ---- Initial error ----
  printErrorStats("Initial reprojection error", config.use_gpu);
  std::cout << std::endl;

  // ---- Solver config summary ----
  std::cout << "Solver configuration:" << std::endl;
  std::cout << "  Backend:        " << (config.use_gpu ? "GPU" : "CPU")
            << std::endl;
  std::cout << "  Loss function:  " << lossTypeStr(config.solver.loss_type);
  if (config.solver.loss_type != backend::solver::LossType::TRIVIAL) {
    std::cout << " (param=" << config.solver.loss_param << ")";
  }
  std::cout << std::endl;
  std::cout << "  Max iterations: " << config.solver.max_iterations
            << std::endl;
  std::cout << "  Initial lambda: " << config.solver.initial_lambda
            << std::endl;
  std::cout << std::endl;

  // ---- Snapshot before optimization (for OpenCV visualization) ----
  viz::BASnapshot snapshot_before;
  if (!config.viz_output_path.empty()) {
    snapshot_before = viz::BASnapshot::capture(problem_);
  }

  // ---- Rerun visualization: log initial state ----
  rerun_viz::BAVisualizer rr_viz;
  std::vector<double> initial_points_copy;
  std::vector<double> initial_cameras_copy;
  if (config.rerun.enabled()) {
    // Save a copy of the initial state before the solver modifies it in-place
    initial_points_copy = problem_.memory_map->scene_points;
    initial_cameras_copy = problem_.memory_map->observers;
    if (rr_viz.init(config.rerun)) {
      rr_viz.logPoints("initial", initial_points_copy.data(),
                       static_cast<int>(problem_.points.size()));
      rr_viz.logCameras("initial", initial_cameras_copy.data(),
                        static_cast<int>(problem_.cameras.size()));
    }
  }

  // ---- Run solver ----
  std::cout << std::string(60, '-') << std::endl;
  std::cout << "Optimizing..." << std::endl;
  std::cout << std::string(60, '-') << std::endl;

  auto start = std::chrono::high_resolution_clock::now();

  backend::solver::LMResult result;
  if (config.use_gpu) {
    result = backend::solver::solveLM_GPU(problem_, config.solver);
  } else {
    result = backend::solver::solveLM(problem_, config.solver);
  }

  auto end = std::chrono::high_resolution_clock::now();
  double elapsed_ms =
      std::chrono::duration<double, std::milli>(end - start).count();

  // ---- Results ----
  printResult(result, elapsed_ms);
  std::cout << std::endl;

  // ---- Final error (L2, regardless of loss function used) ----
  printErrorStats("Final reprojection error (L2)", config.use_gpu);

  // ---- OpenCV visualization (before/after PNG) ----
  if (!config.viz_output_path.empty()) {
    std::cout << std::endl;
    viz::BASnapshot snapshot_after = viz::BASnapshot::capture(problem_);
    viz::BAVisualizer::renderBeforeAfter(snapshot_before, snapshot_after,
                                         config.viz_output_path);
  }

  // ---- Rerun visualization: log final state + solver metrics ----
  if (config.rerun.enabled()) {
    int num_obs = static_cast<int>(problem_.observations.size());
    double initial_rms = std::sqrt(result.initial_cost / num_obs);
    double final_rms = std::sqrt(result.final_cost / num_obs);
    rr_viz.logIteration(0, result.initial_cost, initial_rms,
                        config.solver.initial_lambda);
    rr_viz.logIteration(result.iterations, result.final_cost, final_rms, 0.0);

    // Log optimized state
    rr_viz.logPoints("optimized", problem_.memory_map->scene_points.data(),
                     static_cast<int>(problem_.points.size()));
    rr_viz.logCameras("optimized", problem_.memory_map->observers.data(),
                      static_cast<int>(problem_.cameras.size()));

    // Diagnostic: report how much the 3D data changed
    if (!initial_points_copy.empty()) {
      double max_pt_delta = 0.0, sum_pt_delta = 0.0;
      int n_pts = static_cast<int>(problem_.points.size());
      for (int i = 0; i < n_pts * 3; ++i) {
        double d = std::abs(problem_.memory_map->scene_points[i] -
                            initial_points_copy[i]);
        sum_pt_delta += d;
        if (d > max_pt_delta) max_pt_delta = d;
      }
      double max_cam_delta = 0.0, sum_cam_delta = 0.0;
      int n_cams = static_cast<int>(problem_.cameras.size());
      for (int i = 0; i < n_cams * 9; ++i) {
        double d = std::abs(problem_.memory_map->observers[i] -
                            initial_cameras_copy[i]);
        sum_cam_delta += d;
        if (d > max_cam_delta) max_cam_delta = d;
      }
      std::cout << "\n[viz] 3D change diagnostics:" << std::endl;
      std::cout << "  Points:  max delta = " << max_pt_delta
                << ", mean delta = " << sum_pt_delta / (n_pts * 3) << std::endl;
      std::cout << "  Cameras: max delta = " << max_cam_delta
                << ", mean delta = " << sum_cam_delta / (n_cams * 9)
                << std::endl;
    }
  }
}

void BAPipeline::printProblemStats() const {
  std::cout << "Problem size:" << std::endl;
  std::cout << "  Cameras:      " << problem_.cameras.size() << std::endl;
  std::cout << "  Points:       " << problem_.points.size() << std::endl;
  std::cout << "  Observations: " << problem_.observations.size() << std::endl;

  int total_cam_params = static_cast<int>(problem_.cameras.size()) * 9;
  int total_pt_params = static_cast<int>(problem_.points.size()) * 3;
  std::cout << "  Camera params: " << total_cam_params
            << "  Point params: " << total_pt_params
            << "  Total: " << total_cam_params + total_pt_params << std::endl;
}

void BAPipeline::printErrorStats(const std::string& label, bool gpu) {
  double total_sq_error =
      gpu ? backend::ops::computeProjectionErrorGPU(problem_)
          : backend::ops::computeProjectionErrorCPU(problem_);
  int num_obs = static_cast<int>(problem_.observations.size());
  double rms = std::sqrt(total_sq_error / num_obs);
  double mean_sq = total_sq_error / num_obs;

  std::cout << label << ":" << std::endl;
  std::cout << "  Total squared error: " << total_sq_error << std::endl;
  std::cout << "  Mean squared error:  " << mean_sq << std::endl;
  std::cout << "  RMS error:           " << rms << " px" << std::endl;
}

}  // namespace substral
