#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/functional.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/reduce.h>
#include <thrust/transform_reduce.h>

#include <cmath>

#include "subastral/backend/common.h"
#include "subastral/backend/ops/errors.cuh"
#include "subastral/backend/ops/projection_gpu.cuh"

namespace substral {
namespace backend {
namespace ops {

__global__ void projectionResidual(const double* __restrict__ cameras,
                                   const double* __restrict__ points,
                                   const double* __restrict__ observations,
                                   const int* __restrict__ camera_indices,
                                   const int* __restrict__ point_indices,
                                   int num_observations,
                                   double* residuals_out) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_observations) return;

  int cam_idx = camera_indices[i];
  int pt_idx = point_indices[i];

  const double* cam = &cameras[cam_idx * 9];
  const double* pt = &points[pt_idx * 3];
  const double* obs = &observations[i * 2];

  double predicted[2];
  ops::device::projectPoint(cam, pt, predicted);

  residuals_out[i * 2 + 0] = predicted[0] - obs[0];
  residuals_out[i * 2 + 1] = predicted[1] - obs[1];
}

double computeProjectionErrorGPU(backend::BAProblem& problem) {
  int num_obs = problem.get_num_observations();

  thrust::device_vector<double> d_cameras = problem.memory_map->observers;
  thrust::device_vector<double> d_points = problem.memory_map->scene_points;
  thrust::device_vector<double> d_observations =
      problem.memory_map->observations;
  thrust::device_vector<int> d_cam_indices =
      problem.memory_map->observation_camera_indices;
  thrust::device_vector<int> d_pt_indices =
      problem.memory_map->observation_point_indices;

  thrust::device_vector<double> d_residuals(num_obs * 2);

  int threads = 256;
  int blocks = (num_obs + threads - 1) / threads;

  projectionResidual<<<blocks, threads>>>(
      thrust::raw_pointer_cast(d_cameras.data()),
      thrust::raw_pointer_cast(d_points.data()),
      thrust::raw_pointer_cast(d_observations.data()),
      thrust::raw_pointer_cast(d_cam_indices.data()),
      thrust::raw_pointer_cast(d_pt_indices.data()), num_obs,
      thrust::raw_pointer_cast(d_residuals.data()));

  cudaDeviceSynchronize();

  return thrust::transform_reduce(d_residuals.begin(), d_residuals.end(),
                                  thrust::square<double>(), 0.0,
                                  thrust::plus<double>());
}

}  // namespace ops
}  // namespace backend
}  // namespace substral
