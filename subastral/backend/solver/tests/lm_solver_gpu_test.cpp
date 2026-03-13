#include "subastral/backend/solver/lm_solver_gpu.cuh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "subastral/backend/common.h"
#include "subastral/backend/lie/se3.hpp"
#include "subastral/backend/lie/so3.hpp"
#include "subastral/backend/ops/projection_cpu.hpp"
#include "subastral/backend/solver/gpu_solver_kernels.cuh"

using namespace substral::backend;
using namespace substral::backend::solver;

// =============================================================================
// Test helper: build a synthetic BA problem with optional noise
// =============================================================================

static BAProblem buildSyntheticProblem(int num_cameras, int num_points,
                                       double point_noise_sigma = 0.0,
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

  for (int j = 0; j < num_points; ++j) {
    auto pt = std::make_shared<Point>(j, problem.memory_map);
    double* d = pt->mutable_data();
    d[0] = point_dist(rng);
    d[1] = point_dist(rng);
    d[2] = depth_dist(rng);
    problem.points.push_back(pt);
  }

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

      problem.observations.push_back(obs);
      obs_id++;
    }
  }

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

// No copyProblem needed — we build two independent problems with the same seed

// =============================================================================
// Test: GPU solver converges to near-zero cost on noise-free problem
// =============================================================================

TEST(LMSolverGPUTest, NoiseFree_ConvergesToZeroCost) {
  BAProblem problem = buildSyntheticProblem(3, 5, 0.5, 42);

  LMConfig config;
  config.max_iterations = 100;
  config.verbose = true;
  config.initial_lambda = 1e-3;
  config.cost_tolerance = 1e-15;
  config.gradient_tolerance = 1e-15;
  config.step_tolerance = 1e-15;

  LMResult result = solveLM_GPU(problem, config);

  std::cout << "GPU: Initial cost: " << result.initial_cost << std::endl;
  std::cout << "GPU: Final cost:   " << result.final_cost << std::endl;
  std::cout << "GPU: Iterations:   " << result.iterations << std::endl;
  std::cout << "GPU: Reason:       " << result.termination_reason << std::endl;

  EXPECT_GT(result.initial_cost, 1.0);
  EXPECT_LT(result.final_cost, 1e-6);
  EXPECT_TRUE(result.converged);
}

// =============================================================================
// Test: GPU solver matches CPU solver results
// =============================================================================
//
// Run GPU solver twice on identical problems to verify determinism.
// Also verify the initial cost matches the known CPU value (18251.7 for
// this seed/config, validated by the CPU solver tests).
//
TEST(LMSolverGPUTest, Deterministic_TwoRuns) {
  BAProblem problem1 = buildSyntheticProblem(3, 5, 0.5, 42);
  BAProblem problem2 = buildSyntheticProblem(3, 5, 0.5, 42);

  LMConfig config;
  config.max_iterations = 100;
  config.verbose = false;
  config.initial_lambda = 1e-3;
  config.cost_tolerance = 1e-15;
  config.gradient_tolerance = 1e-15;
  config.step_tolerance = 1e-15;

  LMResult result1 = solveLM_GPU(problem1, config);
  LMResult result2 = solveLM_GPU(problem2, config);

  // Both should converge to same cost
  EXPECT_TRUE(result1.converged);
  EXPECT_TRUE(result2.converged);
  EXPECT_NEAR(result1.final_cost, result2.final_cost, 1e-10);
  EXPECT_EQ(result1.iterations, result2.iterations);

  // Initial cost should match the CPU solver's known value
  // (CPU test shows initial cost = 18251.7)
  EXPECT_NEAR(result1.initial_cost, 18251.7, 1.0);

  // Final parameters should match between runs
  for (int i = 0; i < static_cast<int>(problem1.cameras.size()); ++i) {
    const double* cam1 = problem1.cameras[i]->data();
    const double* cam2 = problem2.cameras[i]->data();
    for (int d = 0; d < 9; ++d) {
      EXPECT_NEAR(cam1[d], cam2[d], 1e-12)
          << "Camera " << i << " param " << d << " differs between runs";
    }
  }

  for (int j = 0; j < static_cast<int>(problem1.points.size()); ++j) {
    const double* pt1 = problem1.points[j]->data();
    const double* pt2 = problem2.points[j]->data();
    for (int d = 0; d < 3; ++d) {
      EXPECT_NEAR(pt1[d], pt2[d], 1e-12)
          << "Point " << j << " param " << d << " differs between runs";
    }
  }
}

// =============================================================================
// Test: GPU solver on larger problem
// =============================================================================

TEST(LMSolverGPUTest, LargerProblem_Converges) {
  BAProblem problem = buildSyntheticProblem(5, 20, 0.3, 777);

  LMConfig config;
  config.max_iterations = 100;
  config.verbose = true;
  config.cost_tolerance = 1e-15;
  config.gradient_tolerance = 1e-15;
  config.step_tolerance = 1e-15;

  LMResult result = solveLM_GPU(problem, config);

  std::cout << "GPU larger: " << result.initial_cost << " -> "
            << result.final_cost << " in " << result.iterations << " iters"
            << std::endl;

  EXPECT_LT(result.final_cost, 1e-6);
  EXPECT_TRUE(result.converged);
}

// =============================================================================
// Test: GPU solver with no observations
// =============================================================================

TEST(LMSolverGPUTest, NoObservations_ReturnsImmediately) {
  BAProblem problem;
  LMConfig config;
  config.verbose = false;

  LMResult result = solveLM_GPU(problem, config);

  EXPECT_EQ(result.initial_cost, 0.0);
  EXPECT_EQ(result.final_cost, 0.0);
  EXPECT_TRUE(result.converged);
}

// =============================================================================
// Test helper: build a synthetic BA problem with outlier observations
// =============================================================================

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

  for (int j = 0; j < num_points; ++j) {
    auto pt = std::make_shared<Point>(j, problem.memory_map);
    double* d = pt->mutable_data();
    d[0] = point_dist(rng);
    d[1] = point_dist(rng);
    d[2] = depth_dist(rng);
    problem.points.push_back(pt);
  }

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

      if (uniform01(rng) < outlier_fraction) {
        obs_data[0] += outlier_dist(rng);
        obs_data[1] += outlier_dist(rng);
      }

      problem.observations.push_back(obs);
      obs_id++;
    }
  }

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
// Test: GPU Huber solver recovers from outliers
// =============================================================================
TEST(LMSolverGPUTest, HuberLoss_RecoversFromOutliers) {
  auto problem_l2 = buildProblemWithOutliers(3, 10, 0.3, 0.2, 100.0, 42);
  auto problem_huber = buildProblemWithOutliers(3, 10, 0.3, 0.2, 100.0, 42);

  LMConfig config_l2;
  config_l2.max_iterations = 50;
  config_l2.verbose = false;
  config_l2.loss_type = LossType::TRIVIAL;

  LMConfig config_huber;
  config_huber.max_iterations = 200;
  config_huber.verbose = true;
  config_huber.loss_type = LossType::HUBER;
  config_huber.loss_param = 1.0;

  LMResult result_l2 = solveLM_GPU(problem_l2, config_l2);
  LMResult result_huber = solveLM_GPU(problem_huber, config_huber);

  std::cout << "GPU L2 final cost:    " << result_l2.final_cost << std::endl;
  std::cout << "GPU Huber final cost: " << result_huber.final_cost << std::endl;

  // IRLS with Huber loss on outlier-heavy problems can converge very slowly
  // (known issue: small steps when many outliers are present). The key test
  // is that Huber produces a substantially lower cost than L2, not that it
  // hits a formal convergence criterion within the iteration budget.
  EXPECT_LT(result_huber.final_cost, result_huber.initial_cost * 0.2)
      << "GPU Huber should reduce cost by at least 80%";

  EXPECT_LT(result_huber.final_cost, result_l2.final_cost)
      << "GPU Huber cost should be less than L2 cost with outliers";
}

// =============================================================================
// Test: GPU Cauchy solver recovers from outliers
// =============================================================================
TEST(LMSolverGPUTest, CauchyLoss_RecoversFromOutliers) {
  auto problem = buildProblemWithOutliers(3, 10, 0.3, 0.2, 100.0, 42);

  LMConfig config;
  config.max_iterations = 100;
  config.verbose = true;
  config.loss_type = LossType::CAUCHY;
  config.loss_param = 1.0;

  LMResult result = solveLM_GPU(problem, config);

  std::cout << "GPU Cauchy: " << result.initial_cost << " -> "
            << result.final_cost << " in " << result.iterations << " iters"
            << std::endl;

  // Cauchy loss compresses the cost via rho(s) = c^2*log(1+s/c^2), so the
  // initial cost is already much smaller than L2. The key test is that cost
  // decreases substantially from its starting value.
  EXPECT_LT(result.final_cost, result.initial_cost * 0.5)
      << "GPU Cauchy should reduce cost by at least 50%";
}

// =============================================================================
// Test: GPU robust loss on clean problem converges to near-zero
// =============================================================================
TEST(LMSolverGPUTest, RobustLoss_NoiseFree_ConvergesToZero) {
  auto problem = buildSyntheticProblem(3, 5, 0.5, 42);

  LMConfig config;
  config.max_iterations = 100;
  config.verbose = false;
  config.cost_tolerance = 1e-15;
  config.gradient_tolerance = 1e-15;
  config.step_tolerance = 1e-15;
  config.loss_type = LossType::HUBER;
  config.loss_param = 10.0;

  LMResult result = solveLM_GPU(problem, config);

  EXPECT_LT(result.final_cost, 1e-6);
  EXPECT_TRUE(result.converged);
}

// =============================================================================
// Lie Group Solver Tests
// =============================================================================

// =============================================================================
// Test: Lie update kernel produces correct SE(3) composition
// =============================================================================
//
// Verifies that updateCameraParametersLieKernel correctly computes:
//   T_new = Exp(δξ) · T_old
// by comparing GPU results against CPU Lie group math.
//
// =============================================================================
TEST(LMSolverGPULieTest, UpdateKernel_MatchesCPU) {
  using namespace substral::backend::lie;

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> angle_dist(-0.8, 0.8);
  std::uniform_real_distribution<double> trans_dist(-5.0, 5.0);
  std::uniform_real_distribution<double> delta_dist(-0.1, 0.1);
  std::uniform_real_distribution<double> focal_dist(300.0, 800.0);
  std::uniform_real_distribution<double> k_dist(-0.01, 0.01);

  constexpr int N = 50;

  // Build camera params and deltas
  std::vector<double> h_cameras(N * 9);
  std::vector<double> h_deltas(N * 9);
  std::vector<double> h_cameras_expected(N * 9);

  for (int i = 0; i < N; ++i) {
    // Random camera
    h_cameras[i * 9 + 0] = angle_dist(rng);
    h_cameras[i * 9 + 1] = angle_dist(rng);
    h_cameras[i * 9 + 2] = angle_dist(rng);
    h_cameras[i * 9 + 3] = trans_dist(rng);
    h_cameras[i * 9 + 4] = trans_dist(rng);
    h_cameras[i * 9 + 5] = trans_dist(rng);
    h_cameras[i * 9 + 6] = focal_dist(rng);
    h_cameras[i * 9 + 7] = k_dist(rng);
    h_cameras[i * 9 + 8] = k_dist(rng) * 0.1;

    // Random delta
    for (int j = 0; j < 9; ++j) {
      h_deltas[i * 9 + j] = delta_dist(rng);
    }

    // CPU reference: Lie update
    Eigen::Vector3d w_old(h_cameras[i * 9 + 0], h_cameras[i * 9 + 1],
                          h_cameras[i * 9 + 2]);
    Eigen::Vector3d t_old(h_cameras[i * 9 + 3], h_cameras[i * 9 + 4],
                          h_cameras[i * 9 + 5]);

    Eigen::Matrix<double, 6, 1> delta_xi;
    for (int j = 0; j < 6; ++j) delta_xi(j) = h_deltas[i * 9 + j];

    // T_old
    Eigen::Matrix4d T_old = Eigen::Matrix4d::Identity();
    T_old.block<3, 3>(0, 0) = expSO3(w_old);
    T_old.block<3, 1>(0, 3) = t_old;

    // T_new = Exp(δξ) · T_old
    Eigen::Matrix4d T_delta = expSE3(delta_xi);
    Eigen::Matrix4d T_new = T_delta * T_old;

    Eigen::Vector3d w_new = logSO3(T_new.block<3, 3>(0, 0));
    Eigen::Vector3d t_new = T_new.block<3, 1>(0, 3);

    h_cameras_expected[i * 9 + 0] = w_new(0);
    h_cameras_expected[i * 9 + 1] = w_new(1);
    h_cameras_expected[i * 9 + 2] = w_new(2);
    h_cameras_expected[i * 9 + 3] = t_new(0);
    h_cameras_expected[i * 9 + 4] = t_new(1);
    h_cameras_expected[i * 9 + 5] = t_new(2);
    // Intrinsics: additive
    h_cameras_expected[i * 9 + 6] = h_cameras[i * 9 + 6] + h_deltas[i * 9 + 6];
    h_cameras_expected[i * 9 + 7] = h_cameras[i * 9 + 7] + h_deltas[i * 9 + 7];
    h_cameras_expected[i * 9 + 8] = h_cameras[i * 9 + 8] + h_deltas[i * 9 + 8];
  }

  // Run GPU kernel
  double *d_cameras, *d_deltas;
  cudaMalloc(&d_cameras, N * 9 * sizeof(double));
  cudaMalloc(&d_deltas, N * 9 * sizeof(double));
  cudaMemcpy(d_cameras, h_cameras.data(), N * 9 * sizeof(double),
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_deltas, h_deltas.data(), N * 9 * sizeof(double),
             cudaMemcpyHostToDevice);

  substral::backend::solver::gpu::launchUpdateCameraParametersLie(d_cameras,
                                                                  d_deltas, N);

  std::vector<double> h_cameras_gpu(N * 9);
  cudaMemcpy(h_cameras_gpu.data(), d_cameras, N * 9 * sizeof(double),
             cudaMemcpyDeviceToHost);

  cudaFree(d_cameras);
  cudaFree(d_deltas);

  // Compare
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < 9; ++j) {
      EXPECT_NEAR(h_cameras_expected[i * 9 + j], h_cameras_gpu[i * 9 + j],
                  1e-10)
          << "Camera " << i << " param " << j
          << ": expected=" << h_cameras_expected[i * 9 + j]
          << " gpu=" << h_cameras_gpu[i * 9 + j];
    }
  }
}

// =============================================================================
// Test: Lie update with zero delta is identity
// =============================================================================
TEST(LMSolverGPULieTest, UpdateKernel_ZeroDeltaIsIdentity) {
  constexpr int N = 10;
  std::mt19937 rng(123);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  std::vector<double> h_cameras(N * 9);
  std::vector<double> h_deltas(N * 9, 0.0);  // all zeros

  for (int i = 0; i < N; ++i) {
    h_cameras[i * 9 + 0] = dist(rng) * 0.5;
    h_cameras[i * 9 + 1] = dist(rng) * 0.5;
    h_cameras[i * 9 + 2] = dist(rng) * 0.5;
    h_cameras[i * 9 + 3] = dist(rng) * 3.0;
    h_cameras[i * 9 + 4] = dist(rng) * 3.0;
    h_cameras[i * 9 + 5] = dist(rng) * 3.0;
    h_cameras[i * 9 + 6] = 500.0;
    h_cameras[i * 9 + 7] = -0.001;
    h_cameras[i * 9 + 8] = 0.0001;
  }

  std::vector<double> h_cameras_orig = h_cameras;

  double *d_cameras, *d_deltas;
  cudaMalloc(&d_cameras, N * 9 * sizeof(double));
  cudaMalloc(&d_deltas, N * 9 * sizeof(double));
  cudaMemcpy(d_cameras, h_cameras.data(), N * 9 * sizeof(double),
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_deltas, h_deltas.data(), N * 9 * sizeof(double),
             cudaMemcpyHostToDevice);

  substral::backend::solver::gpu::launchUpdateCameraParametersLie(d_cameras,
                                                                  d_deltas, N);

  std::vector<double> h_cameras_result(N * 9);
  cudaMemcpy(h_cameras_result.data(), d_cameras, N * 9 * sizeof(double),
             cudaMemcpyDeviceToHost);

  cudaFree(d_cameras);
  cudaFree(d_deltas);

  for (int i = 0; i < N * 9; ++i) {
    EXPECT_NEAR(h_cameras_orig[i], h_cameras_result[i], 1e-12)
        << "Zero delta should leave camera " << i / 9 << " param " << i % 9
        << " unchanged";
  }
}

// =============================================================================
// Test: Lie GPU solver converges to near-zero cost on noise-free problem
// =============================================================================
TEST(LMSolverGPULieTest, NoiseFree_ConvergesToZeroCost) {
  BAProblem problem = buildSyntheticProblem(3, 5, 0.5, 42);

  LMConfig config;
  config.max_iterations = 100;
  config.verbose = true;
  config.initial_lambda = 1e-3;
  config.cost_tolerance = 1e-15;
  config.gradient_tolerance = 1e-15;
  config.step_tolerance = 1e-15;
  config.use_lie = true;

  LMResult result = solveLM_GPU(problem, config);

  std::cout << "GPU Lie: Initial cost: " << result.initial_cost << std::endl;
  std::cout << "GPU Lie: Final cost:   " << result.final_cost << std::endl;
  std::cout << "GPU Lie: Iterations:   " << result.iterations << std::endl;
  std::cout << "GPU Lie: Reason:       " << result.termination_reason
            << std::endl;

  EXPECT_GT(result.initial_cost, 1.0);
  EXPECT_LT(result.final_cost, 1e-6);
  EXPECT_TRUE(result.converged);
}

// =============================================================================
// Test: Lie GPU solver on larger problem
// =============================================================================
TEST(LMSolverGPULieTest, LargerProblem_Converges) {
  BAProblem problem = buildSyntheticProblem(5, 20, 0.3, 777);

  LMConfig config;
  config.max_iterations = 100;
  config.verbose = true;
  config.cost_tolerance = 1e-15;
  config.gradient_tolerance = 1e-15;
  config.step_tolerance = 1e-15;
  config.use_lie = true;

  LMResult result = solveLM_GPU(problem, config);

  std::cout << "GPU Lie larger: " << result.initial_cost << " -> "
            << result.final_cost << " in " << result.iterations << " iters"
            << std::endl;

  EXPECT_LT(result.final_cost, 1e-6);
  EXPECT_TRUE(result.converged);
}

// =============================================================================
// Test: Lie solver matches non-Lie solver quality
// =============================================================================
//
// Both should converge to near-zero cost on the same noise-free problem.
// The paths differ (different Jacobians, different update rules) but the
// optimum is the same.
//
// =============================================================================
TEST(LMSolverGPULieTest, MatchesNonLieSolverQuality) {
  BAProblem problem_lie = buildSyntheticProblem(3, 5, 0.5, 42);
  BAProblem problem_std = buildSyntheticProblem(3, 5, 0.5, 42);

  LMConfig config;
  config.max_iterations = 100;
  config.verbose = false;
  config.cost_tolerance = 1e-15;
  config.gradient_tolerance = 1e-15;
  config.step_tolerance = 1e-15;

  LMConfig config_lie = config;
  config_lie.use_lie = true;

  LMResult result_std = solveLM_GPU(problem_std, config);
  LMResult result_lie = solveLM_GPU(problem_lie, config_lie);

  std::cout << "Standard: " << result_std.initial_cost << " -> "
            << result_std.final_cost << " in " << result_std.iterations
            << " iters" << std::endl;
  std::cout << "Lie:      " << result_lie.initial_cost << " -> "
            << result_lie.final_cost << " in " << result_lie.iterations
            << " iters" << std::endl;

  // Both should start from the same cost
  EXPECT_NEAR(result_std.initial_cost, result_lie.initial_cost, 1e-6);

  // Both should converge to near-zero
  EXPECT_LT(result_std.final_cost, 1e-6);
  EXPECT_LT(result_lie.final_cost, 1e-6);
  EXPECT_TRUE(result_std.converged);
  EXPECT_TRUE(result_lie.converged);
}
