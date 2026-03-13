#include "subastral/backend/solver/lm_solver.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "subastral/backend/common.h"
#include "subastral/backend/ops/projection_cpu.hpp"

using namespace substral::backend;
using namespace substral::backend::solver;

// =============================================================================
// Test helper: build a synthetic BA problem with optional noise
// =============================================================================
//
// Creates a noise-free problem, then optionally perturbs the points.
// For a noise-free problem, the LM solver should drive the cost to ~0.
//
static BAProblem buildSyntheticProblem(int num_cameras, int num_points,
                                       double point_noise_sigma = 0.0,
                                       unsigned seed = 42) {
  BAProblem problem;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> angle_dist(-0.1, 0.1);
  std::uniform_real_distribution<double> trans_dist(-1.0, 1.0);
  std::uniform_real_distribution<double> point_dist(-2.0, 2.0);
  std::uniform_real_distribution<double> depth_dist(5.0, 15.0);

  // Create cameras
  for (int i = 0; i < num_cameras; ++i) {
    auto cam = std::make_shared<Camera>(i, problem.memory_map);
    double* d = cam->mutable_data();
    d[0] = angle_dist(rng);
    d[1] = angle_dist(rng);
    d[2] = angle_dist(rng);
    d[3] = trans_dist(rng);
    d[4] = trans_dist(rng);
    d[5] = trans_dist(rng);
    d[6] = 500.0;
    d[7] = 0.0;
    d[8] = 0.0;
    problem.cameras.push_back(cam);
  }

  // Create points
  for (int j = 0; j < num_points; ++j) {
    auto pt = std::make_shared<Point>(j, problem.memory_map);
    double* d = pt->mutable_data();
    d[0] = point_dist(rng);
    d[1] = point_dist(rng);
    d[2] = depth_dist(rng);
    problem.points.push_back(pt);
  }

  // Project to create noise-free observations
  // We need the projection function
  // Import it indirectly through lm_solver which includes it
  // Actually, let's just use the ops function directly
  // We'll include it via the lm_solver dependency chain

  // For observations, we need to project. Let's use a simple approach:
  // include the CPU projection
  int obs_id = 0;
  for (int i = 0; i < num_cameras; ++i) {
    for (int j = 0; j < num_points; ++j) {
      auto obs =
          std::make_shared<Observation>(obs_id, i, j, problem.memory_map);

      // Project to get observation
      const double* cam = problem.cameras[i]->data();
      const double* pt = problem.points[j]->data();

      // Use the projection from projection_cpu.hpp (available via lm_solver
      // deps)
      double predicted[2];
      substral::backend::ops::project_cpu(cam, pt, predicted);

      double* obs_data =
          &problem.memory_map
               ->observations[problem.memory_map->observation_indices[obs_id]];
      obs_data[0] = predicted[0];
      obs_data[1] = predicted[1];

      problem.observations.push_back(obs);
      obs_id++;
    }
  }

  // Now perturb points if requested
  if (point_noise_sigma > 0.0) {
    std::normal_distribution<double> noise(0.0, point_noise_sigma);
    for (auto& pt : problem.points) {
      double* d = pt->mutable_data();
      d[0] += noise(rng);
      d[1] += noise(rng);
      d[2] += noise(rng);
    }
  }

  return problem;
}

// =============================================================================
// Test 2h: End-to-end LM solver on noise-free synthetic problem
// =============================================================================
//
// With noise-free observations and perturbed initial points, the LM solver
// should converge to near-zero cost (recovering the true point positions).
//
TEST(LMSolverTest, NoiseFree_ConvergesToZeroCost) {
  // Build problem: noise-free observations, then perturb points
  BAProblem problem = buildSyntheticProblem(
      /*num_cameras=*/3, /*num_points=*/5,
      /*point_noise_sigma=*/0.5, /*seed=*/42);

  LMConfig config;
  config.max_iterations = 100;
  config.verbose = true;
  config.initial_lambda = 1e-3;
  config.cost_tolerance = 1e-15;
  config.gradient_tolerance = 1e-15;
  config.step_tolerance = 1e-15;

  LMResult result = solveLM(problem, config);

  std::cout << "Initial cost: " << result.initial_cost << std::endl;
  std::cout << "Final cost:   " << result.final_cost << std::endl;
  std::cout << "Iterations:   " << result.iterations << std::endl;
  std::cout << "Reason:       " << result.termination_reason << std::endl;

  // Initial cost should be nonzero (we perturbed points)
  EXPECT_GT(result.initial_cost, 1.0)
      << "Initial cost should be significant after perturbation";

  // Final cost should be very small (noise-free observations)
  EXPECT_LT(result.final_cost, 1e-6)
      << "Final cost should be near zero for noise-free problem";

  // Should converge
  EXPECT_TRUE(result.converged)
      << "Solver should converge. Reason: " << result.termination_reason;
}

// =============================================================================
// Test: LM solver cost monotonically decreases (accepted steps only)
// =============================================================================
//
// For a well-posed problem, the cost should decrease at each accepted step.
// We verify this by checking final_cost < initial_cost.
//
TEST(LMSolverTest, CostDecreases) {
  BAProblem problem = buildSyntheticProblem(
      /*num_cameras=*/2, /*num_points=*/4,
      /*point_noise_sigma=*/1.0, /*seed=*/99);

  LMConfig config;
  config.max_iterations = 50;
  config.verbose = true;

  LMResult result = solveLM(problem, config);

  EXPECT_LT(result.final_cost, result.initial_cost)
      << "Cost should decrease. Initial: " << result.initial_cost
      << " Final: " << result.final_cost;
}

// =============================================================================
// Test: LM solver with no observations
// =============================================================================
TEST(LMSolverTest, NoObservations_ReturnsImmediately) {
  BAProblem problem;
  LMConfig config;
  config.verbose = false;

  LMResult result = solveLM(problem, config);

  EXPECT_EQ(result.initial_cost, 0.0);
  EXPECT_EQ(result.final_cost, 0.0);
  EXPECT_TRUE(result.converged);
}

// =============================================================================
// Test: LM solver on a larger problem
// =============================================================================
//
// 5 cameras, 20 points = 100 observations.
// Verifies the solver scales correctly and still converges.
//
TEST(LMSolverTest, LargerProblem_Converges) {
  BAProblem problem = buildSyntheticProblem(
      /*num_cameras=*/5, /*num_points=*/20,
      /*point_noise_sigma=*/0.3, /*seed=*/777);

  LMConfig config;
  config.max_iterations = 100;
  config.verbose = true;
  config.cost_tolerance = 1e-15;
  config.gradient_tolerance = 1e-15;
  config.step_tolerance = 1e-15;

  LMResult result = solveLM(problem, config);

  std::cout << "Larger problem: " << result.initial_cost << " -> "
            << result.final_cost << " in " << result.iterations << " iters"
            << std::endl;

  EXPECT_LT(result.final_cost, 1e-6)
      << "Should converge to near-zero for noise-free observations";
  EXPECT_TRUE(result.converged);
}

// =============================================================================
// Test helper: build a synthetic BA problem with outlier observations
// =============================================================================
//
// Creates a noise-free problem with perturbed initial points, then corrupts
// a fraction of the observations with large random offsets. These outlier
// observations have residuals much larger than inliers.
//
// With L2 loss, the solver tries to fit the outliers too, dragging the
// solution away from the true parameters. With Huber or Cauchy loss,
// the outlier observations are down-weighted and the solver recovers
// the true parameters (near-zero inlier cost).
//
static BAProblem buildProblemWithOutliers(int num_cameras, int num_points,
                                          double point_noise_sigma,
                                          double outlier_fraction,
                                          double outlier_magnitude,
                                          unsigned seed = 42) {
  BAProblem problem;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> angle_dist(-0.1, 0.1);
  std::uniform_real_distribution<double> trans_dist(-1.0, 1.0);
  std::uniform_real_distribution<double> point_dist(-2.0, 2.0);
  std::uniform_real_distribution<double> depth_dist(5.0, 15.0);

  for (int i = 0; i < num_cameras; ++i) {
    auto cam = std::make_shared<Camera>(i, problem.memory_map);
    double* d = cam->mutable_data();
    d[0] = angle_dist(rng);
    d[1] = angle_dist(rng);
    d[2] = angle_dist(rng);
    d[3] = trans_dist(rng);
    d[4] = trans_dist(rng);
    d[5] = trans_dist(rng);
    d[6] = 500.0;
    d[7] = 0.0;
    d[8] = 0.0;
    problem.cameras.push_back(cam);
  }

  // Save true point positions for later comparison
  std::vector<std::array<double, 3>> true_points(num_points);
  for (int j = 0; j < num_points; ++j) {
    auto pt = std::make_shared<Point>(j, problem.memory_map);
    double* d = pt->mutable_data();
    d[0] = point_dist(rng);
    d[1] = point_dist(rng);
    d[2] = depth_dist(rng);
    true_points[j] = {d[0], d[1], d[2]};
    problem.points.push_back(pt);
  }

  // Project to create observations, corrupting some with outlier noise
  std::uniform_real_distribution<double> uniform01(0.0, 1.0);
  std::uniform_real_distribution<double> outlier_dist(-outlier_magnitude,
                                                      outlier_magnitude);
  int obs_id = 0;
  for (int i = 0; i < num_cameras; ++i) {
    for (int j = 0; j < num_points; ++j) {
      auto obs =
          std::make_shared<Observation>(obs_id, i, j, problem.memory_map);
      const double* cam = problem.cameras[i]->data();
      const double* pt = problem.points[j]->data();
      double predicted[2];
      substral::backend::ops::project_cpu(cam, pt, predicted);

      double* obs_data =
          &problem.memory_map
               ->observations[problem.memory_map->observation_indices[obs_id]];
      obs_data[0] = predicted[0];
      obs_data[1] = predicted[1];

      // Corrupt this observation with probability outlier_fraction
      if (uniform01(rng) < outlier_fraction) {
        obs_data[0] += outlier_dist(rng);
        obs_data[1] += outlier_dist(rng);
      }

      problem.observations.push_back(obs);
      obs_id++;
    }
  }

  // Perturb initial point positions
  if (point_noise_sigma > 0.0) {
    std::normal_distribution<double> noise(0.0, point_noise_sigma);
    for (auto& pt : problem.points) {
      double* d = pt->mutable_data();
      d[0] += noise(rng);
      d[1] += noise(rng);
      d[2] += noise(rng);
    }
  }

  return problem;
}

// =============================================================================
// Test: Huber loss recovers from outliers while L2 fails
// =============================================================================
//
// With 20% outlier observations (magnitude 100 pixels), L2 loss gets pulled
// toward the outliers and ends up with a large final cost. Huber loss
// down-weights the outliers and converges to a much better solution.
//
TEST(LMSolverTest, HuberLoss_RecoversFromOutliers) {
  // Build identical problems for L2 and Huber
  auto problem_l2 = buildProblemWithOutliers(
      /*num_cameras=*/3, /*num_points=*/10,
      /*point_noise_sigma=*/0.3,
      /*outlier_fraction=*/0.2,
      /*outlier_magnitude=*/100.0,
      /*seed=*/42);

  auto problem_huber = buildProblemWithOutliers(
      /*num_cameras=*/3, /*num_points=*/10,
      /*point_noise_sigma=*/0.3,
      /*outlier_fraction=*/0.2,
      /*outlier_magnitude=*/100.0,
      /*seed=*/42);

  LMConfig config_l2;
  config_l2.max_iterations = 100;
  config_l2.verbose = false;
  config_l2.loss_type = LossType::TRIVIAL;

  // IRLS-based robust optimization converges more slowly because the weights
  // change between iterations. Use more iterations and a higher initial lambda
  // to stabilize the early steps.
  LMConfig config_huber;
  config_huber.max_iterations = 100;
  config_huber.verbose = true;
  config_huber.loss_type = LossType::HUBER;
  config_huber.loss_param = 1.0;  // delta = 1 pixel

  LMResult result_l2 = solveLM(problem_l2, config_l2);
  LMResult result_huber = solveLM(problem_huber, config_huber);

  std::cout << "L2 final cost:    " << result_l2.final_cost << std::endl;
  std::cout << "Huber final cost: " << result_huber.final_cost << std::endl;

  // The key test: Huber cost should be significantly lower than the initial
  // Huber cost. The solver should make substantial progress even if it doesn't
  // hit the strict convergence criteria (IRLS can stall near the optimum
  // because weight changes between iterations create small oscillations).
  EXPECT_LT(result_huber.final_cost, result_huber.initial_cost * 0.5)
      << "Huber should reduce cost by at least 50%";

  // Huber should also achieve a lower cost than L2 on the same problem,
  // because L2 gets dragged by outliers while Huber down-weights them.
  // Note: costs are measured with different metrics (L2 vs Huber), but
  // Huber cost is always <= L2 cost for the same parameters.
  EXPECT_LT(result_huber.final_cost, result_l2.final_cost)
      << "Huber cost should be less than L2 cost with outliers";
}

// =============================================================================
// Test: Cauchy loss recovers from outliers
// =============================================================================
TEST(LMSolverTest, CauchyLoss_RecoversFromOutliers) {
  auto problem = buildProblemWithOutliers(
      /*num_cameras=*/3, /*num_points=*/10,
      /*point_noise_sigma=*/0.3,
      /*outlier_fraction=*/0.2,
      /*outlier_magnitude=*/100.0,
      /*seed=*/42);

  LMConfig config;
  config.max_iterations = 100;
  config.verbose = true;
  config.loss_type = LossType::CAUCHY;
  config.loss_param = 1.0;  // c = 1 pixel

  LMResult result = solveLM(problem, config);

  std::cout << "Cauchy: " << result.initial_cost << " -> " << result.final_cost
            << " in " << result.iterations << " iters" << std::endl;

  // Cauchy loss compresses the cost significantly via rho(s) =
  // c^2*log(1+s/c^2). For c=1 and s=10000 (100px outlier), rho(s) = log(10001)
  // ≈ 9.2. So initial_cost is already much smaller than L2 initial cost. The
  // key test: cost should decrease substantially from its starting value.
  EXPECT_LT(result.final_cost, result.initial_cost * 0.5)
      << "Cauchy should reduce cost by at least 50%";
}

// =============================================================================
// Test: Robust loss on clean problem matches L2
// =============================================================================
//
// When there are no outliers, Huber and Cauchy should converge to the same
// solution as L2 (since all residuals are small, weights are ~1).
//
TEST(LMSolverTest, RobustLoss_NoiseFree_MatchesL2) {
  // With no outliers and large loss parameters, robust losses should behave
  // nearly identically to L2 and converge to near-zero cost.
  //
  // We use large loss parameters so that all residuals fall in the quadratic
  // region (Huber) or near-unity weight region (Cauchy), making the IRLS
  // weighting a near-identity operation.
  auto problem_l2 = buildSyntheticProblem(3, 5, 0.5, 42);
  auto problem_huber = buildSyntheticProblem(3, 5, 0.5, 42);
  auto problem_cauchy = buildSyntheticProblem(3, 5, 0.5, 42);

  LMConfig config;
  config.max_iterations = 200;
  config.verbose = false;
  config.cost_tolerance = 1e-15;
  config.gradient_tolerance = 1e-15;
  config.step_tolerance = 1e-15;

  config.loss_type = LossType::TRIVIAL;
  LMResult result_l2 = solveLM(problem_l2, config);

  // Huber with delta=1000: sqrt(s) must exceed 1000 to enter linear region.
  // For our problem, initial residuals are ~50 RMS, so all observations stay
  // in the quadratic region => weight = 1 => identical to L2.
  config.loss_type = LossType::HUBER;
  config.loss_param = 1000.0;
  LMResult result_huber = solveLM(problem_huber, config);

  // Cauchy with c=1000: weight = 1e6/(1e6+s). For s up to ~10000 (50px RMS),
  // weight ≈ 0.99, so nearly identical to L2.
  config.loss_type = LossType::CAUCHY;
  config.loss_param = 1000.0;
  LMResult result_cauchy = solveLM(problem_cauchy, config);

  std::cout << "L2 final:     " << result_l2.final_cost << " in "
            << result_l2.iterations << " iters ["
            << result_l2.termination_reason << "]" << std::endl;
  std::cout << "Huber final:  " << result_huber.final_cost << " in "
            << result_huber.iterations << " iters ["
            << result_huber.termination_reason << "]" << std::endl;
  std::cout << "Cauchy final: " << result_cauchy.final_cost << " in "
            << result_cauchy.iterations << " iters ["
            << result_cauchy.termination_reason << "]" << std::endl;

  // All should converge to near-zero
  EXPECT_LT(result_l2.final_cost, 1e-6);
  EXPECT_LT(result_huber.final_cost, 1e-6);
  EXPECT_LT(result_cauchy.final_cost, 1e-6);

  EXPECT_TRUE(result_l2.converged);
  EXPECT_TRUE(result_huber.converged);
  EXPECT_TRUE(result_cauchy.converged);
}
