#pragma once

#include <cuda_runtime.h>

#include "subastral/backend/ops/errors.h"

namespace substral {
namespace backend {
namespace ops {

// GPU kernel: computes per-observation residuals on device.
__global__ void projectionResidual(const double* __restrict__ cameras,
                                   const double* __restrict__ points,
                                   const double* __restrict__ observations,
                                   const int* __restrict__ camera_indices,
                                   const int* __restrict__ point_indices,
                                   int num_observations, double* residuals_out);

}  // namespace ops
}  // namespace backend
}  // namespace substral
