#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/functional.h>
#include <thrust/transform_reduce.h>

#include <cmath>

#include "subastral/backend/common.h"
#include "subastral/backend/ops/jacobians.cuh"
#include "subastral/backend/ops/project_with_jacobian_gpu.cuh"
#include "subastral/backend/ops/project_with_jacobian_lie_gpu.cuh"

namespace substral {
namespace backend {
namespace ops {

// =============================================================================
// CUDA Kernel: Fused residual + Jacobian computation
// =============================================================================
//
// One thread per observation. Each thread:
//   1. Reads its camera and point data
//   2. Calls projectWithJacobian to get predicted pixel + both Jacobians
//   3. Computes residual = predicted - observed
//   4. Writes residual, J_cam, J_pt to global memory
//
// Note: The Jacobians are of the projection function, not the residual.
// Since residual = predicted - observed, and observed is constant:
//   d(residual)/d(params) = d(predicted)/d(params)
// So the Jacobians are the same.
//
// =============================================================================

__global__ void projectionResidualAndJacobian(
    const double* __restrict__ cameras,
    const double* __restrict__ points,
    const double* __restrict__ observations,
    const int* __restrict__ camera_indices,
    const int* __restrict__ point_indices,
    int num_observations,
    double* residuals_out,
    double* J_cameras_out,
    double* J_points_out) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_observations) return;

  int cam_idx = camera_indices[i];
  int pt_idx = point_indices[i];

  const double* cam = &cameras[cam_idx * 9];
  const double* pt = &points[pt_idx * 3];
  const double* obs = &observations[i * 2];

  double predicted[2];
  double J_cam[18];  // 2x9 row-major
  double J_pt[6];    // 2x3 row-major

  device::projectWithJacobian(cam, pt, predicted, J_cam, J_pt);

  // Residual = predicted - observed
  residuals_out[i * 2 + 0] = predicted[0] - obs[0];
  residuals_out[i * 2 + 1] = predicted[1] - obs[1];

  // Write Jacobians to global memory
  for (int j = 0; j < 18; ++j) {
    J_cameras_out[i * 18 + j] = J_cam[j];
  }
  for (int j = 0; j < 6; ++j) {
    J_points_out[i * 6 + j] = J_pt[j];
  }
}

// =============================================================================
// CUDA Kernel: Fused residual + Lie group Jacobian computation
// =============================================================================
//
// Identical to projectionResidualAndJacobian but uses the SE(3)
// left-perturbation Jacobian. The Lie Jacobian is simpler to compute
// (no Rodrigues derivative needed) and is the correct Jacobian for
// the exponential map update T_new = Exp(δξ) · T_old.
//
// =============================================================================

__global__ void projectionResidualAndJacobianLie(
    const double* __restrict__ cameras,
    const double* __restrict__ points,
    const double* __restrict__ observations,
    const int* __restrict__ camera_indices,
    const int* __restrict__ point_indices,
    int num_observations,
    double* residuals_out,
    double* J_cameras_out,
    double* J_points_out) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_observations) return;

  int cam_idx = camera_indices[i];
  int pt_idx = point_indices[i];

  const double* cam = &cameras[cam_idx * 9];
  const double* pt = &points[pt_idx * 3];
  const double* obs = &observations[i * 2];

  double predicted[2];
  double J_cam[18];  // 2x9 row-major
  double J_pt[6];    // 2x3 row-major

  device::projectWithJacobianLie(cam, pt, predicted, J_cam, J_pt);

  // Residual = predicted - observed
  residuals_out[i * 2 + 0] = predicted[0] - obs[0];
  residuals_out[i * 2 + 1] = predicted[1] - obs[1];

  // Write Jacobians to global memory
  for (int j = 0; j < 18; ++j) {
    J_cameras_out[i * 18 + j] = J_cam[j];
  }
  for (int j = 0; j < 6; ++j) {
    J_points_out[i * 6 + j] = J_pt[j];
  }
}

// =============================================================================
// Host wrapper: GPU
// =============================================================================

double computeResidualsAndJacobiansGPU(
    backend::BAProblem& problem,
    double* d_residuals,
    double* d_J_cameras,
    double* d_J_points) {
  int num_obs = problem.get_num_observations();

  // Transfer problem data to device
  thrust::device_vector<double> d_cameras = problem.memory_map->observers;
  thrust::device_vector<double> d_points = problem.memory_map->scene_points;
  thrust::device_vector<double> d_observations =
      problem.memory_map->observations;
  thrust::device_vector<int> d_cam_indices =
      problem.memory_map->observation_camera_indices;
  thrust::device_vector<int> d_pt_indices =
      problem.memory_map->observation_point_indices;

  int threads = 256;
  int blocks = (num_obs + threads - 1) / threads;

  projectionResidualAndJacobian<<<blocks, threads>>>(
      thrust::raw_pointer_cast(d_cameras.data()),
      thrust::raw_pointer_cast(d_points.data()),
      thrust::raw_pointer_cast(d_observations.data()),
      thrust::raw_pointer_cast(d_cam_indices.data()),
      thrust::raw_pointer_cast(d_pt_indices.data()),
      num_obs,
      d_residuals,
      d_J_cameras,
      d_J_points);

  cudaDeviceSynchronize();

  // Compute total squared error from residuals for convenience
  thrust::device_ptr<double> res_ptr(d_residuals);
  double total_sq_error = thrust::transform_reduce(
      res_ptr, res_ptr + num_obs * 2,
      thrust::square<double>(), 0.0, thrust::plus<double>());

  return total_sq_error;
}

// =============================================================================
// Host wrapper: GPU (Lie group Jacobians)
// =============================================================================

double computeResidualsAndJacobiansLieGPU(
    backend::BAProblem& problem,
    double* d_residuals,
    double* d_J_cameras,
    double* d_J_points) {
  int num_obs = problem.get_num_observations();

  thrust::device_vector<double> d_cameras = problem.memory_map->observers;
  thrust::device_vector<double> d_points = problem.memory_map->scene_points;
  thrust::device_vector<double> d_observations =
      problem.memory_map->observations;
  thrust::device_vector<int> d_cam_indices =
      problem.memory_map->observation_camera_indices;
  thrust::device_vector<int> d_pt_indices =
      problem.memory_map->observation_point_indices;

  int threads = 256;
  int blocks = (num_obs + threads - 1) / threads;

  projectionResidualAndJacobianLie<<<blocks, threads>>>(
      thrust::raw_pointer_cast(d_cameras.data()),
      thrust::raw_pointer_cast(d_points.data()),
      thrust::raw_pointer_cast(d_observations.data()),
      thrust::raw_pointer_cast(d_cam_indices.data()),
      thrust::raw_pointer_cast(d_pt_indices.data()),
      num_obs,
      d_residuals,
      d_J_cameras,
      d_J_points);

  cudaDeviceSynchronize();

  thrust::device_ptr<double> res_ptr(d_residuals);
  double total_sq_error = thrust::transform_reduce(
      res_ptr, res_ptr + num_obs * 2,
      thrust::square<double>(), 0.0, thrust::plus<double>());

  return total_sq_error;
}

}  // namespace ops
}  // namespace backend
}  // namespace substral
