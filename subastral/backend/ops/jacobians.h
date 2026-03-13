#pragma once

#include "subastral/backend/common.h"

namespace substral {
namespace backend {
namespace ops {

// CPU version: computes residuals and Jacobians for all observations.
// Output arrays must be pre-allocated by the caller.
//
// Returns total squared error (for convenience).
double computeResidualsAndJacobiansCPU(
    backend::BAProblem& problem,
    double* residuals,  // host ptr, size: num_obs * 2
    double* J_cameras,  // host ptr, size: num_obs * 18
    double* J_points);  // host ptr, size: num_obs * 6

// GPU version: computes residuals and Jacobians on device.
// Output pointers must be pre-allocated device memory.
//
// Returns total squared error (for convenience).
double computeResidualsAndJacobiansGPU(
    backend::BAProblem& problem,
    double* d_residuals,  // device ptr, size: num_obs * 2
    double* d_J_cameras,  // device ptr, size: num_obs * 18
    double* d_J_points);  // device ptr, size: num_obs * 6

// GPU version with SE(3) Lie group Jacobians.
// Same interface as computeResidualsAndJacobiansGPU but uses the
// left-perturbation Jacobian (projectWithJacobianLie).
double computeResidualsAndJacobiansLieGPU(
    backend::BAProblem& problem,
    double* d_residuals,  // device ptr, size: num_obs * 2
    double* d_J_cameras,  // device ptr, size: num_obs * 18
    double* d_J_points);  // device ptr, size: num_obs * 6

}  // namespace ops
}  // namespace backend
}  // namespace substral
