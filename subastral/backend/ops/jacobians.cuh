#pragma once

#include <cuda_runtime.h>

#include "subastral/backend/ops/jacobians.h"

namespace substral {
namespace backend {
namespace ops {

// =============================================================================
// Batch Jacobian Computation -- GPU Kernel
// =============================================================================
//
// One thread per observation. Each thread:
//   1. Reads its camera and point data
//   2. Calls projectWithJacobian to get predicted pixel + both Jacobians
//   3. Computes residual = predicted - observed
//   4. Writes residual, J_cam, J_pt to global memory
//
// For N observations, outputs:
//   - residuals:  N*2 doubles   (predicted - observed)
//   - J_cameras:  N*18 doubles  (2x9 Jacobian per observation, row-major)
//   - J_points:   N*6 doubles   (2x3 Jacobian per observation, row-major)
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
    double* J_points_out);

// =============================================================================
// Batch Lie Group Jacobian Computation -- GPU Kernel
// =============================================================================
//
// Identical to projectionResidualAndJacobian but uses the SE(3)
// left-perturbation Jacobian (projectWithJacobianLie).
//
// J_cam layout (2×9, row-major):
//   cols 0-2: ∂(px,py)/∂δφ  (rotation perturbation via -[P_cam]×)
//   cols 3-5: ∂(px,py)/∂δρ  (translation perturbation = C)
//   cols 6-8: ∂(px,py)/∂(f, k1, k2)  (intrinsics, additive)
//
// J_pt layout (2×3, row-major):
//   ∂(px,py)/∂P_world = C · R  (same as non-Lie version)
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
    double* J_points_out);

}  // namespace ops
}  // namespace backend
}  // namespace substral
