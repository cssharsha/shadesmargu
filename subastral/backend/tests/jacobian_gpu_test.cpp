#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "subastral/backend/common.h"
#include "subastral/backend/ops/jacobians.h"
#include "subastral/backend/ops/project_with_jacobian_lie_cpu.hpp"

namespace substral {
namespace backend {
namespace ops {

// =============================================================================
// GPU vs CPU Consistency Test
// =============================================================================
//
// Constructs a small synthetic BAProblem, runs both the GPU and CPU batch
// Jacobian functions, and verifies that all outputs match within tolerance.
//
// This tests:
//   1. The CUDA kernel produces the same residuals as CPU
//   2. The CUDA kernel produces the same J_cam as CPU
//   3. The CUDA kernel produces the same J_pt as CPU
//   4. The total squared error matches
// =============================================================================

static constexpr double kTolerance = 1e-10;

class JacobianGPUTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Build a small synthetic problem:
    //   2 cameras, 3 points, 4 observations
    problem_.memory_map = std::make_shared<FactorGraphMemoryMap>();

    // Camera 0: identity rotation, small translation
    auto cam0 = std::make_shared<Camera>(0, problem_.memory_map);
    double* c0 = cam0->mutable_data();
    c0[0] = 0.1;
    c0[1] = -0.2;
    c0[2] = 0.3;  // ω
    c0[3] = 0.5;
    c0[4] = -1.0;
    c0[5] = 0.2;     // t
    c0[6] = 500.0;   // f
    c0[7] = -0.001;  // k1
    c0[8] = 0.0001;  // k2
    problem_.cameras.push_back(cam0);

    // Camera 1: different pose
    auto cam1 = std::make_shared<Camera>(1, problem_.memory_map);
    double* c1 = cam1->mutable_data();
    c1[0] = 0.5;
    c1[1] = 0.3;
    c1[2] = -0.4;
    c1[3] = -1.0;
    c1[4] = 0.5;
    c1[5] = 1.0;
    c1[6] = 600.0;
    c1[7] = -0.002;
    c1[8] = 0.0005;
    problem_.cameras.push_back(cam1);

    // 3 points
    auto pt0 = std::make_shared<Point>(0, problem_.memory_map);
    double* p0 = pt0->mutable_data();
    p0[0] = 1.0;
    p0[1] = 2.0;
    p0[2] = 10.0;
    problem_.points.push_back(pt0);

    auto pt1 = std::make_shared<Point>(1, problem_.memory_map);
    double* p1 = pt1->mutable_data();
    p1[0] = -3.0;
    p1[1] = 1.5;
    p1[2] = 8.0;
    problem_.points.push_back(pt1);

    auto pt2 = std::make_shared<Point>(2, problem_.memory_map);
    double* p2 = pt2->mutable_data();
    p2[0] = 0.5;
    p2[1] = -1.0;
    p2[2] = 15.0;
    problem_.points.push_back(pt2);

    // 4 observations (cam_id, pt_id, obs_x, obs_y)
    struct ObsData {
      int cam, pt;
      double x, y;
    };
    ObsData obs_data[] = {
        {0, 0, -50.0, -100.0},
        {0, 1, 180.0, -70.0},
        {1, 0, -30.0, -120.0},
        {1, 2, -20.0, 40.0},
    };

    for (int i = 0; i < 4; ++i) {
      auto obs = std::make_shared<Observation>(
          i, obs_data[i].cam, obs_data[i].pt, problem_.memory_map);
      problem_.observations.push_back(obs);

      int idx = problem_.memory_map->observation_indices[i];
      problem_.memory_map->observations[idx] = obs_data[i].x;
      problem_.memory_map->observations[idx + 1] = obs_data[i].y;
    }
  }

  BAProblem problem_;
};

TEST_F(JacobianGPUTest, GPUMatchesCPU) {
  int num_obs = problem_.get_num_observations();
  ASSERT_EQ(num_obs, 4);

  // ---- CPU computation ----
  std::vector<double> cpu_residuals(num_obs * 2);
  std::vector<double> cpu_J_cameras(num_obs * 18);
  std::vector<double> cpu_J_points(num_obs * 6);

  double cpu_error = computeResidualsAndJacobiansCPU(
      problem_, cpu_residuals.data(), cpu_J_cameras.data(),
      cpu_J_points.data());

  // ---- GPU computation ----
  double* d_residuals;
  double* d_J_cameras;
  double* d_J_points;
  cudaMalloc(&d_residuals, num_obs * 2 * sizeof(double));
  cudaMalloc(&d_J_cameras, num_obs * 18 * sizeof(double));
  cudaMalloc(&d_J_points, num_obs * 6 * sizeof(double));

  double gpu_error = computeResidualsAndJacobiansGPU(problem_, d_residuals,
                                                     d_J_cameras, d_J_points);

  // Copy results back to host
  std::vector<double> gpu_residuals(num_obs * 2);
  std::vector<double> gpu_J_cameras(num_obs * 18);
  std::vector<double> gpu_J_points(num_obs * 6);

  cudaMemcpy(gpu_residuals.data(), d_residuals, num_obs * 2 * sizeof(double),
             cudaMemcpyDeviceToHost);
  cudaMemcpy(gpu_J_cameras.data(), d_J_cameras, num_obs * 18 * sizeof(double),
             cudaMemcpyDeviceToHost);
  cudaMemcpy(gpu_J_points.data(), d_J_points, num_obs * 6 * sizeof(double),
             cudaMemcpyDeviceToHost);

  cudaFree(d_residuals);
  cudaFree(d_J_cameras);
  cudaFree(d_J_points);

  // ---- Compare ----

  // Total squared error
  EXPECT_NEAR(cpu_error, gpu_error, 1e-6) << "Total squared error mismatch";

  // Residuals
  for (int i = 0; i < num_obs * 2; ++i) {
    EXPECT_NEAR(cpu_residuals[i], gpu_residuals[i], kTolerance)
        << "Residual mismatch at index " << i;
  }

  // Camera Jacobians
  for (int i = 0; i < num_obs * 18; ++i) {
    EXPECT_NEAR(cpu_J_cameras[i], gpu_J_cameras[i], kTolerance)
        << "J_camera mismatch at index " << i << " (obs=" << i / 18
        << ", entry=" << i % 18 << ")";
  }

  // Point Jacobians
  for (int i = 0; i < num_obs * 6; ++i) {
    EXPECT_NEAR(cpu_J_points[i], gpu_J_points[i], kTolerance)
        << "J_point mismatch at index " << i << " (obs=" << i / 6
        << ", entry=" << i % 6 << ")";
  }
}

// =============================================================================
// GPU vs CPU Consistency Test — Lie Group Jacobian
// =============================================================================
//
// Uses the host wrapper computeResidualsAndJacobiansLieGPU which calls the
// projectionResidualAndJacobianLie kernel, and compares against CPU Lie
// Jacobian (projectWithJacobianLieCPU) computed per-observation.
//
// =============================================================================

TEST_F(JacobianGPUTest, LieGPUMatchesCPU) {
  int num_obs = problem_.get_num_observations();
  ASSERT_EQ(num_obs, 4);

  // ---- CPU computation (Lie Jacobian) ----
  std::vector<double> cpu_residuals(num_obs * 2);
  std::vector<double> cpu_J_cameras(num_obs * 18);
  std::vector<double> cpu_J_points(num_obs * 6);

  for (int i = 0; i < num_obs; ++i) {
    auto& observation = problem_.observations[i];
    const double* cam = problem_.cameras[observation->get_camera_id()]->data();
    const double* point = problem_.points[observation->get_point_id()]->data();

    double predicted[2];
    double* J_cam = &cpu_J_cameras[i * 18];
    double* J_pt = &cpu_J_points[i * 6];

    projectWithJacobianLieCPU(cam, point, predicted, J_cam, J_pt);

    cpu_residuals[i * 2 + 0] = predicted[0] - observation->data()[0];
    cpu_residuals[i * 2 + 1] = predicted[1] - observation->data()[1];
  }

  // ---- GPU computation (Lie Jacobian via host wrapper) ----
  double* d_residuals;
  double* d_J_cameras;
  double* d_J_points;
  cudaMalloc(&d_residuals, num_obs * 2 * sizeof(double));
  cudaMalloc(&d_J_cameras, num_obs * 18 * sizeof(double));
  cudaMalloc(&d_J_points, num_obs * 6 * sizeof(double));

  computeResidualsAndJacobiansLieGPU(problem_, d_residuals, d_J_cameras,
                                     d_J_points);

  // Copy results back to host
  std::vector<double> gpu_residuals(num_obs * 2);
  std::vector<double> gpu_J_cameras(num_obs * 18);
  std::vector<double> gpu_J_points(num_obs * 6);

  cudaMemcpy(gpu_residuals.data(), d_residuals, num_obs * 2 * sizeof(double),
             cudaMemcpyDeviceToHost);
  cudaMemcpy(gpu_J_cameras.data(), d_J_cameras, num_obs * 18 * sizeof(double),
             cudaMemcpyDeviceToHost);
  cudaMemcpy(gpu_J_points.data(), d_J_points, num_obs * 6 * sizeof(double),
             cudaMemcpyDeviceToHost);

  cudaFree(d_residuals);
  cudaFree(d_J_cameras);
  cudaFree(d_J_points);

  // ---- Compare ----

  // Residuals (should be identical — same projection pipeline)
  for (int i = 0; i < num_obs * 2; ++i) {
    EXPECT_NEAR(cpu_residuals[i], gpu_residuals[i], kTolerance)
        << "Lie residual mismatch at index " << i;
  }

  // Camera Jacobians (Lie version)
  for (int i = 0; i < num_obs * 18; ++i) {
    EXPECT_NEAR(cpu_J_cameras[i], gpu_J_cameras[i], kTolerance)
        << "Lie J_camera mismatch at index " << i << " (obs=" << i / 18
        << ", entry=" << i % 18 << ")";
  }

  // Point Jacobians (same for Lie and non-Lie: C · R)
  for (int i = 0; i < num_obs * 6; ++i) {
    EXPECT_NEAR(cpu_J_points[i], gpu_J_points[i], kTolerance)
        << "Lie J_point mismatch at index " << i << " (obs=" << i / 6
        << ", entry=" << i % 6 << ")";
  }
}

// =============================================================================
// Test: Lie Jacobian differs from non-Lie in pose columns (sanity check)
// =============================================================================
//
// The Lie Jacobian (cols 0-5) should differ from the global-param Jacobian
// because they use different perturbation models. The intrinsic columns (6-8)
// and J_pt should be identical. This is a sanity check that we're actually
// computing something different, not just duplicating the old kernel.
//
// =============================================================================

TEST_F(JacobianGPUTest, LieJacobianDiffersFromGlobalInPoseColumns) {
  int num_obs = problem_.get_num_observations();
  ASSERT_EQ(num_obs, 4);

  // Run both via host wrappers
  double *d_res1, *d_Jc1, *d_Jp1;
  double *d_res2, *d_Jc2, *d_Jp2;
  cudaMalloc(&d_res1, num_obs * 2 * sizeof(double));
  cudaMalloc(&d_Jc1, num_obs * 18 * sizeof(double));
  cudaMalloc(&d_Jp1, num_obs * 6 * sizeof(double));
  cudaMalloc(&d_res2, num_obs * 2 * sizeof(double));
  cudaMalloc(&d_Jc2, num_obs * 18 * sizeof(double));
  cudaMalloc(&d_Jp2, num_obs * 6 * sizeof(double));

  // Non-Lie
  computeResidualsAndJacobiansGPU(problem_, d_res1, d_Jc1, d_Jp1);
  // Lie
  computeResidualsAndJacobiansLieGPU(problem_, d_res2, d_Jc2, d_Jp2);

  std::vector<double> h_Jc1(num_obs * 18), h_Jc2(num_obs * 18);
  std::vector<double> h_Jp1(num_obs * 6), h_Jp2(num_obs * 6);
  std::vector<double> h_res1(num_obs * 2), h_res2(num_obs * 2);

  cudaMemcpy(h_res1.data(), d_res1, num_obs * 2 * sizeof(double),
             cudaMemcpyDeviceToHost);
  cudaMemcpy(h_res2.data(), d_res2, num_obs * 2 * sizeof(double),
             cudaMemcpyDeviceToHost);
  cudaMemcpy(h_Jc1.data(), d_Jc1, num_obs * 18 * sizeof(double),
             cudaMemcpyDeviceToHost);
  cudaMemcpy(h_Jc2.data(), d_Jc2, num_obs * 18 * sizeof(double),
             cudaMemcpyDeviceToHost);
  cudaMemcpy(h_Jp1.data(), d_Jp1, num_obs * 6 * sizeof(double),
             cudaMemcpyDeviceToHost);
  cudaMemcpy(h_Jp2.data(), d_Jp2, num_obs * 6 * sizeof(double),
             cudaMemcpyDeviceToHost);

  cudaFree(d_res1);
  cudaFree(d_Jc1);
  cudaFree(d_Jp1);
  cudaFree(d_res2);
  cudaFree(d_Jc2);
  cudaFree(d_Jp2);

  // Residuals should be identical (same projection pipeline)
  for (int i = 0; i < num_obs * 2; ++i) {
    EXPECT_NEAR(h_res1[i], h_res2[i], 1e-12)
        << "Residuals should match between Lie and non-Lie at index " << i;
  }

  // Pose columns (0-5) should differ for at least some observations
  bool pose_differs = false;
  for (int obs = 0; obs < num_obs; ++obs) {
    for (int col = 0; col < 6; ++col) {
      // Row 0
      if (std::abs(h_Jc1[obs * 18 + col] - h_Jc2[obs * 18 + col]) > 1e-6) {
        pose_differs = true;
      }
      // Row 1
      if (std::abs(h_Jc1[obs * 18 + 9 + col] - h_Jc2[obs * 18 + 9 + col]) >
          1e-6) {
        pose_differs = true;
      }
    }
  }
  EXPECT_TRUE(pose_differs)
      << "Lie and non-Lie Jacobians should differ in pose columns (0-5)";

  // Intrinsic columns (6-8) should be identical
  for (int obs = 0; obs < num_obs; ++obs) {
    for (int col = 6; col < 9; ++col) {
      EXPECT_NEAR(h_Jc1[obs * 18 + col], h_Jc2[obs * 18 + col], 1e-12)
          << "Intrinsic cols should match at obs=" << obs << " col=" << col;
      EXPECT_NEAR(h_Jc1[obs * 18 + 9 + col], h_Jc2[obs * 18 + 9 + col], 1e-12)
          << "Intrinsic cols should match at obs=" << obs << " col=" << col;
    }
  }

  // J_pt should be identical (C · R is the same for both parameterizations)
  for (int i = 0; i < num_obs * 6; ++i) {
    EXPECT_NEAR(h_Jp1[i], h_Jp2[i], 1e-12)
        << "J_pt should match between Lie and non-Lie at index " << i;
  }
}

}  // namespace ops
}  // namespace backend
}  // namespace substral
