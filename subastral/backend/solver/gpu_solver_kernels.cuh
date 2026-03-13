#pragma once

#include <cuda_runtime.h>

#include "subastral/backend/solver/loss_function_gpu.cuh"

namespace substral {
namespace backend {
namespace solver {
namespace gpu {

// =============================================================================
// GPU Solver Kernel Constants
// =============================================================================
//
// These mirror the CPU constants in schur.hpp but are usable in device code.
//

static constexpr int CAM_DIM = 9;
static constexpr int PT_DIM = 3;
static constexpr int RES_DIM = 2;

// Flattened block sizes
static constexpr int U_BLOCK = CAM_DIM * CAM_DIM;   // 81
static constexpr int V_BLOCK = PT_DIM * PT_DIM;      // 9
static constexpr int W_BLOCK = CAM_DIM * PT_DIM;     // 27
static constexpr int J_CAM_BLOCK = RES_DIM * CAM_DIM; // 18
static constexpr int J_PT_BLOCK = RES_DIM * PT_DIM;   // 6

// =============================================================================
// Kernel 1: Normal Equation Accumulation
// =============================================================================
//
// One thread per observation. For observation k relating camera i to point j:
//
//   J_ck: 2x9 Jacobian w.r.t. camera i  (row-major, at J_cameras[k*18])
//   J_pk: 2x3 Jacobian w.r.t. point j   (row-major, at J_points[k*6])
//   r_k:  2x1 residual                   (at residuals[k*2])
//
// Accumulates (via atomicAdd):
//   U[i]   += J_ck^T * J_ck        (9x9, stored as 81 doubles per camera)
//   V[j]   += J_pk^T * J_pk        (3x3, stored as 9 doubles per point)
//   g_c[i] += J_ck^T * r_k         (9x1, stored as 9 doubles per camera)
//   g_p[j] += J_pk^T * r_k         (3x1, stored as 3 doubles per point)
//
// Direct write (no atomics):
//   W[k]    = J_ck^T * J_pk        (9x3, stored as 27 doubles per obs)
//
// Memory layout:
//   d_U:   [num_cameras * 81]  -- flattened row-major 9x9 per camera
//   d_V:   [num_points * 9]    -- flattened row-major 3x3 per point
//   d_W:   [num_obs * 27]      -- flattened row-major 9x3 per observation
//   d_g_c: [num_cameras * 9]   -- 9x1 per camera
//   d_g_p: [num_points * 3]    -- 3x1 per point
//
// =============================================================================

__global__ void accumulateNormalEquationsKernel(
    const double* __restrict__ residuals,     // [num_obs * 2]
    const double* __restrict__ J_cameras,     // [num_obs * 18]
    const double* __restrict__ J_points,      // [num_obs * 6]
    const int* __restrict__ camera_indices,   // [num_obs]
    const int* __restrict__ point_indices,    // [num_obs]
    int num_obs,
    double* d_U,    // [num_cameras * 81]  -- atomicAdd
    double* d_V,    // [num_points * 9]    -- atomicAdd
    double* d_W,    // [num_obs * 27]      -- direct write
    double* d_g_c,  // [num_cameras * 9]   -- atomicAdd
    double* d_g_p,  // [num_points * 3]    -- atomicAdd
    LossType loss_type = LossType::TRIVIAL,
    double loss_param = 1.0
);

// =============================================================================
// Kernel 2: Initialize Schur system with U blocks and g_c
// =============================================================================
//
// Writes U[i] into the diagonal block S(i,i) and g_c[i] into rhs(i).
// One thread per camera.
//
// Must be called BEFORE schurComplementKernel (which subtracts).
//
// =============================================================================

__global__ void initSchurWithUKernel(
    const double* __restrict__ d_U,    // [num_cameras * 81]
    const double* __restrict__ d_g_c,  // [num_cameras * 9]
    int num_cameras,
    int total_cam_dim,                 // num_cameras * 9
    double* d_S,    // [total_cam_dim * total_cam_dim]
    double* d_rhs   // [total_cam_dim]
);

// =============================================================================
// Kernel 3: Schur Complement Formation (point elimination)
// =============================================================================
//
// One thread per point. For point j, collects all observations seeing it
// via the CSR structure (point_obs_offsets, point_obs_list) and computes:
//
//   V_j^{-1} via inline 3x3 Cholesky
//   E_k = W[k] * V_j^{-1}  for each obs k seeing j
//
//   S(cam(k), cam(l)) -= E_k * W[l]^T   for all pairs (k,l) seeing j
//   rhs(cam(k))       -= E_k * g_p[j]    for each k seeing j
//
// Uses atomicAdd on S and rhs since multiple points may contribute to
// the same camera-camera block.
//
// CSR structure:
//   point_obs_offsets[j]:   start index in point_obs_list for point j
//   point_obs_offsets[j+1]: end index (exclusive)
//   point_obs_list[idx]:    observation index
//
// =============================================================================

__global__ void schurComplementKernel(
    const double* __restrict__ d_V,       // [num_points * 9]
    const double* __restrict__ d_W,       // [num_obs * 27]
    const double* __restrict__ d_g_p,     // [num_points * 3]
    const int* __restrict__ camera_indices,      // [num_obs]
    const int* __restrict__ point_obs_offsets,    // [num_points + 1]
    const int* __restrict__ point_obs_list,       // [num_obs]
    int num_points,
    int total_cam_dim,                             // num_cameras * 9
    double* d_S,     // [total_cam_dim * total_cam_dim] -- atomicAdd
    double* d_rhs    // [total_cam_dim]                 -- atomicAdd
);

// =============================================================================
// Kernel 4: Apply LM Damping to S diagonal
// =============================================================================
//
// S(d,d) += lambda * max(S(d,d), min_diag)
// One thread per diagonal element.
//
// =============================================================================

__global__ void applyLMDampingKernel(
    double* d_S,          // [total_cam_dim * total_cam_dim]
    int total_cam_dim,
    double lambda,
    double min_diag       // e.g. 1e-6
);

// =============================================================================
// Kernel 4b: Apply LM Damping to U diagonal blocks (copy + damp)
// =============================================================================
//
// One thread per camera. Copies U[i] → U_damped[i], then adds LM damping
// to the 9 diagonal entries of each 9×9 block:
//
//   U_damped[i](d,d) += lambda * max(U[i](d,d), min_diag)   for d = 0..8
//
// The undamped U is preserved for lambda retries in the inner loop.
//
// =============================================================================

__global__ void applyDampingToUKernel(
    const double* __restrict__ d_U,      // [num_cameras * 81] undamped
    double* d_U_damped,                   // [num_cameras * 81] output
    int num_cameras,
    double lambda,
    double min_diag
);

// =============================================================================
// Kernel 4c: Apply LM Damping to V diagonal blocks (copy + damp)
// =============================================================================
//
// One thread per point. Copies V[j] → V_damped[j], then adds LM damping
// to the 3 diagonal entries of each 3×3 block:
//
//   V_damped[j](d,d) += lambda * max(V[j](d,d), min_diag)   for d = 0..2
//
// This ensures V_damped is always well-conditioned for inversion in the
// Schur complement kernel, eliminating Cholesky failures from ill-conditioned
// V blocks.
//
// =============================================================================

__global__ void applyDampingToVKernel(
    const double* __restrict__ d_V,      // [num_points * 9] undamped
    double* d_V_damped,                   // [num_points * 9] output
    int num_points,
    double lambda,
    double min_diag
);

// =============================================================================
// Kernel 5: Back-Substitution
// =============================================================================
//
// One thread per point. Computes:
//   delta_p_j = -V_j^{-1} * (g_p_j + sum_{k seeing j} W[k]^T * delta_c_{cam(k)})
//
// =============================================================================

__global__ void backSubstituteKernel(
    const double* __restrict__ d_V,       // [num_points * 9]
    const double* __restrict__ d_W,       // [num_obs * 27]
    const double* __restrict__ d_g_p,     // [num_points * 3]
    const double* __restrict__ d_delta_cameras,  // [num_cameras * 9]
    const int* __restrict__ camera_indices,       // [num_obs]
    const int* __restrict__ point_obs_offsets,    // [num_points + 1]
    const int* __restrict__ point_obs_list,       // [num_obs]
    int num_points,
    double* d_delta_points   // [num_points * 3]
);

// =============================================================================
// Kernel 6: Parameter Update
// =============================================================================
//
// cam[i*9 + d] += delta_cameras[i*9 + d]   for all i, d
// pt[j*3 + d]  += delta_points[j*3 + d]    for all j, d
//
// =============================================================================

__global__ void updateParametersKernel(
    double* d_params,                           // cameras or points (contiguous)
    const double* __restrict__ d_delta,         // same size
    int total_elements                           // num_cameras*9 or num_points*3
);

// =============================================================================
// Kernel 6b: Lie Group Camera Parameter Update
// =============================================================================
//
// For the SE(3) Lie group parameterization, camera pose updates use the
// exponential map instead of additive updates:
//
//   T_new = Exp(δξ) · T_old
//
// where δξ = (δφ[3], δρ[3]) is the 6D pose perturbation from the solver,
// and T_old is constructed from the current (ω, t) parameters.
//
// One thread per camera. For camera i:
//   1. Read ω_old = cam[i*9 + 0..2], t_old = cam[i*9 + 3..5]
//   2. Read δξ = delta[i*9 + 0..5]  (first 6 of the 9-dim delta)
//   3. R_old = Exp_SO3(ω_old)
//   4. (R_new, t_new) = Exp_SE3(δξ) · (R_old, t_old)
//   5. ω_new = Log_SO3(R_new)
//   6. Write ω_new, t_new back to cam[i*9 + 0..5]
//   7. Additively update intrinsics: cam[i*9 + 6..8] += delta[i*9 + 6..8]
//
// =============================================================================

__global__ void updateCameraParametersLieKernel(
    double* d_cameras,                          // [num_cameras * 9]
    const double* __restrict__ d_delta_cameras, // [num_cameras * 9]
    int num_cameras
);

// =============================================================================
// Kernel 7: Per-observation squared residual (for cost computation)
// =============================================================================
//
// Computes sq_residuals[i] = rx^2 + ry^2 for observation i.
// The total cost = 0.5 * sum(sq_residuals) is done via thrust::reduce.
//
// This reuses the projectionResidualAndJacobian kernel from jacobians.cu
// for the projection, but we also need a lightweight cost-only kernel
// that doesn't compute Jacobians (cheaper for accept/reject evaluation).
//
// =============================================================================

__global__ void computeSquaredResidualsKernel(
    const double* __restrict__ d_cameras,
    const double* __restrict__ d_points,
    const double* __restrict__ d_observations,
    const int* __restrict__ camera_indices,
    const int* __restrict__ point_indices,
    int num_obs,
    double* d_sq_residuals,   // [num_obs] -- each entry = rho(rx^2 + ry^2)
    LossType loss_type = LossType::TRIVIAL,
    double loss_param = 1.0
);

// =============================================================================
// Host wrapper for Lie camera update kernel
// =============================================================================
//
// Launches updateCameraParametersLieKernel with default block size.
// Callable from .cpp files (no <<<>>> syntax needed).
//
void launchUpdateCameraParametersLie(
    double* d_cameras,
    const double* d_delta_cameras,
    int num_cameras);

}  // namespace gpu
}  // namespace solver
}  // namespace backend
}  // namespace substral
