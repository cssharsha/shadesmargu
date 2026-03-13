#pragma once

#include <cuda_runtime.h>

#include "subastral/backend/lie/se3_gpu.cuh"
#include "subastral/backend/solver/loss_function_gpu.cuh"

namespace substral {
namespace backend {
namespace solver {
namespace pg_gpu {

// =============================================================================
// Pose-Graph GPU Kernel Constants
// =============================================================================

static constexpr int POSE_DIM = 7;    // (x, y, z, qx, qy, qz, qw)
static constexpr int TANGENT_DIM = 6; // se(3) perturbation (ω, v)
static constexpr int RES_DIM = 6;     // residual in se(3)
static constexpr int INFO_DIM = 36;   // 6×6 info matrix, row-major

// =============================================================================
// Device helpers: quaternion ↔ rotation matrix
// =============================================================================

// Quaternion (qx, qy, qz, qw) → 3×3 rotation matrix (row-major)
__device__ inline void quatToRotation(const double* q, double* R) {
  double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
  R[0] = 1.0 - 2.0 * (qy * qy + qz * qz);
  R[1] = 2.0 * (qx * qy - qz * qw);
  R[2] = 2.0 * (qx * qz + qy * qw);
  R[3] = 2.0 * (qx * qy + qz * qw);
  R[4] = 1.0 - 2.0 * (qx * qx + qz * qz);
  R[5] = 2.0 * (qy * qz - qx * qw);
  R[6] = 2.0 * (qx * qz - qy * qw);
  R[7] = 2.0 * (qy * qz + qx * qw);
  R[8] = 1.0 - 2.0 * (qx * qx + qy * qy);
}

// 3×3 rotation matrix (row-major) → quaternion (qx, qy, qz, qw)
__device__ inline void rotationToQuat(const double* R, double* q) {
  double trace = R[0] + R[4] + R[8];
  if (trace > 0.0) {
    double s = 0.5 / sqrt(trace + 1.0);
    q[3] = 0.25 / s;
    q[0] = (R[7] - R[5]) * s;
    q[1] = (R[2] - R[6]) * s;
    q[2] = (R[3] - R[1]) * s;
  } else if (R[0] > R[4] && R[0] > R[8]) {
    double s = 2.0 * sqrt(1.0 + R[0] - R[4] - R[8]);
    q[3] = (R[7] - R[5]) / s;
    q[0] = 0.25 * s;
    q[1] = (R[1] + R[3]) / s;
    q[2] = (R[2] + R[6]) / s;
  } else if (R[4] > R[8]) {
    double s = 2.0 * sqrt(1.0 + R[4] - R[0] - R[8]);
    q[3] = (R[2] - R[6]) / s;
    q[0] = (R[1] + R[3]) / s;
    q[1] = 0.25 * s;
    q[2] = (R[5] + R[7]) / s;
  } else {
    double s = 2.0 * sqrt(1.0 + R[8] - R[0] - R[4]);
    q[3] = (R[3] - R[1]) / s;
    q[0] = (R[2] + R[6]) / s;
    q[1] = (R[5] + R[7]) / s;
    q[2] = 0.25 * s;
  }
  double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (norm > 1e-15) {
    q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
  }
}

__device__ inline void poseToRT(const double* pose, double* R, double* t) {
  t[0] = pose[0]; t[1] = pose[1]; t[2] = pose[2];
  quatToRotation(pose + 3, R);
}

__device__ inline void rtToPose(const double* R, const double* t, double* pose) {
  pose[0] = t[0]; pose[1] = t[1]; pose[2] = t[2];
  rotationToQuat(R, pose + 3);
}

// =============================================================================
// Device helpers: 6×6 linear algebra
// =============================================================================

__device__ inline void mat6x6_vec(const double* A, const double* x, double* y) {
  for (int i = 0; i < 6; ++i) {
    double sum = 0.0;
    for (int j = 0; j < 6; ++j) sum += A[i * 6 + j] * x[j];
    y[i] = sum;
  }
}

__device__ inline void mat6x6_mul(const double* A, const double* B, double* C) {
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 6; ++k) sum += A[i * 6 + k] * B[k * 6 + j];
      C[i * 6 + j] = sum;
    }
}

// =============================================================================
// Device helper: Adjoint of SE(3)
// =============================================================================
__device__ inline void adjointSE3(const double* R, const double* t, double* Ad) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      Ad[i * 6 + j] = R[i * 3 + j];
  for (int i = 0; i < 3; ++i)
    for (int j = 3; j < 6; ++j)
      Ad[i * 6 + j] = 0.0;
  double tHat[9], tHatR[9];
  lie::device::hat3(t, tHat);
  lie::device::mat3x3_mul(tHat, R, tHatR);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      Ad[(i + 3) * 6 + j] = tHatR[i * 3 + j];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      Ad[(i + 3) * 6 + (j + 3)] = R[i * 3 + j];
}

// =============================================================================
// Kernel 1: Compute per-edge residuals
// =============================================================================
//
// e_k = log(T̄_ij^{-1} · T_i^{-1} · T_j)  ∈ ℝ⁶
// One thread per edge.
//
__global__ void computePoseGraphResidualsKernel(
    const double* __restrict__ d_poses,
    const double* __restrict__ d_measurements,
    const int* __restrict__ d_edge_from_idx,
    const int* __restrict__ d_edge_to_idx,
    int num_edges,
    double* d_residuals);

// =============================================================================
// Kernel 2: Compute per-edge analytical Jacobians (left perturbation)
// =============================================================================
//
// For left perturbation T_i ← exp(δξ_i) · T_i, the residual
// e = log(T̄^{-1} · T_i^{-1} · T_j) has Jacobians:
//
//   ∂e/∂ξ_i = -J_r^{-1}(e) · Ad(T_j^{-1})   (6×6)
//   ∂e/∂ξ_j =  J_r^{-1}(e) · Ad(T_j^{-1})   (6×6)
//
// where J_r^{-1}(ξ) = J_l^{-1}(-ξ) (right inverse Jacobian of SE(3)).
//
// Derivation: left-perturbing T_i gives E(ε) = E_0 · exp(-Ad(T_j^{-1})·ε),
// which is a right perturbation of E_0. BCH gives de/dε via J_r^{-1}(e).
// Similarly for T_j. See pose_graph_kernels.cu for the full derivation.
//
// One thread per edge.
//
__global__ void computePoseGraphJacobiansKernel(
    const double* __restrict__ d_poses,
    const int* __restrict__ d_edge_from_idx,
    const int* __restrict__ d_edge_to_idx,
    int num_edges,
    const double* __restrict__ d_residuals,
    double* d_J_i,
    double* d_J_j);

// =============================================================================
// Kernel 3: Accumulate Hessian and gradient into CSR values
// =============================================================================
//
// For each edge k, accumulates 6×6 blocks into the sparse Hessian
// stored in scalar CSR format. The block offsets into the CSR values
// array are precomputed on CPU.
//
// d_block_offsets[num_edges * 4]: for each edge, the offset into
//   d_csr_values where the (ii, ij, ji, jj) blocks start.
//   -1 means skip (one endpoint is fixed).
//
// One thread per edge. Uses atomicAdd.
//
__global__ void accumulatePoseGraphHessianCSRKernel(
    const double* __restrict__ d_residuals,
    const double* __restrict__ d_J_i,
    const double* __restrict__ d_J_j,
    const double* __restrict__ d_info_matrices,
    const int* __restrict__ d_edge_var_i,
    const int* __restrict__ d_edge_var_j,
    const int* __restrict__ d_block_offsets,
    int num_edges,
    int num_free_poses,
    double* d_csr_values,
    double* d_b);

// =============================================================================
// Kernel 4: Apply LM damping to CSR Hessian diagonal
// =============================================================================
//
// For each free pose, adds lambda * max(H(d,d), min_diag) to the 6
// diagonal entries of its diagonal block.
//
// d_diag_offsets[num_free_poses]: offset into d_csr_values for the
//   start of each diagonal 6×6 block.
//
__global__ void applyPoseGraphDampingCSRKernel(
    double* d_csr_values,
    const int* __restrict__ d_diag_offsets,
    int num_free_poses,
    int block_stride,
    double lambda,
    double min_diag);

// =============================================================================
// Kernel 5: Update poses via SE(3) exponential map
// =============================================================================
//
// T_i ← exp(δξ_i) · T_i for each free pose.
// One thread per pose.
//
__global__ void updatePosesKernel(
    double* d_poses,
    const double* __restrict__ d_delta,
    const int* __restrict__ d_vertex_idx_map,
    int num_poses);

// =============================================================================
// Kernel 6: Compute per-edge Mahalanobis cost
// =============================================================================
//
// cost_k = e_k^T · Ω_k · e_k
// One thread per edge.
//
__global__ void computePoseGraphCostKernel(
    const double* __restrict__ d_residuals,
    const double* __restrict__ d_info_matrices,
    int num_edges,
    double* d_costs);

// =============================================================================
// Kernel 13: Compute per-edge IRLS weight for robust loss
// =============================================================================
//
// For each edge k, computes the squared Mahalanobis distance:
//   s_k = e_k^T * Omega_k * e_k
// Then evaluates the IRLS weight:
//   w_k = rho'(s_k)
//
// One thread per edge.
//
__global__ void computeEdgeWeightsKernel(
    const double* __restrict__ d_residuals,
    const double* __restrict__ d_info_matrices,
    int num_edges,
    gpu::LossType loss_type,
    double loss_param,
    double* d_weights);

// =============================================================================
// Kernel 14: Compute per-edge robust cost
// =============================================================================
//
// For each edge k, computes:
//   cost_k = rho(e_k^T * Omega_k * e_k)
// instead of the raw squared Mahalanobis distance.
//
// One thread per edge.
//
__global__ void computePoseGraphRobustCostKernel(
    const double* __restrict__ d_residuals,
    const double* __restrict__ d_info_matrices,
    int num_edges,
    gpu::LossType loss_type,
    double loss_param,
    double* d_costs);

// =============================================================================
// Kernel 7: Symmetric SpMV — y = A * x (A stored as lower-triangle CSR)
// =============================================================================
//
// For each row i, iterates over the lower-triangle entries (i, j) where j <= i.
// Diagonal entries (j == i) contribute val * x[i] to y[i].
// Off-diagonal entries (j < i) contribute val * x[j] to y[i] (direct)
// and val * x[i] to y[j] (symmetric, via atomicAdd).
//
// y must be zeroed before calling this kernel.
// One thread per row.
//
__global__ void symmetricSpMVKernel(
    const int* __restrict__ row_ptr,
    const int* __restrict__ col_ind,
    const double* __restrict__ values,
    const double* __restrict__ x,
    double* y,
    int num_rows);

// =============================================================================
// Kernel 8: Extract and invert 6x6 diagonal blocks (Jacobi preconditioner)
// =============================================================================
//
// For each free pose, extracts the 6x6 diagonal block from the damped CSR
// Hessian and computes its inverse via Gauss-Jordan elimination.
// Stores the 36-element inverse block contiguously.
//
// One thread per free pose.
//
__global__ void extractAndInvertDiagBlocksKernel(
    const double* __restrict__ csr_values,
    const int* __restrict__ diag_offsets,
    int num_free_poses,
    double* inv_blocks);

// =============================================================================
// Kernel 9: Apply block-diagonal preconditioner — z = M^{-1} * r
// =============================================================================
//
// For each free pose, multiplies the 6x6 inverse block by the corresponding
// 6-element segment of r to produce z.
//
// One thread per free pose.
//
__global__ void applyBlockPreconditionerKernel(
    const double* __restrict__ inv_blocks,
    const double* __restrict__ r_vec,
    int num_free_poses,
    double* z);

// =============================================================================
// Kernel 10: Vector axpy — y = alpha * x + y
// =============================================================================
__global__ void axpyKernel(double alpha, const double* __restrict__ x,
                           double* y, int n);

// =============================================================================
// Kernel 11: Vector xpby — y = x + beta * y
// =============================================================================
//
// Used for the PCG direction update: p = z + beta * p
// Reads x, reads+writes y in-place.
//
__global__ void xpbyKernel(const double* __restrict__ x, double beta,
                           double* y, int n);

// =============================================================================
// Kernel 12: Vector negate — y = -x
// =============================================================================
__global__ void negateKernel(const double* __restrict__ x, double* y, int n);

// =============================================================================
// Host wrappers
// =============================================================================

void launchComputePoseGraphResiduals(
    const double* d_poses, const double* d_measurements,
    const int* d_edge_from_idx, const int* d_edge_to_idx,
    int num_edges, double* d_residuals);

void launchComputePoseGraphJacobians(
    const double* d_poses,
    const int* d_edge_from_idx, const int* d_edge_to_idx,
    int num_edges, const double* d_residuals,
    double* d_J_i, double* d_J_j);

void launchAccumulatePoseGraphHessianCSR(
    const double* d_residuals, const double* d_J_i, const double* d_J_j,
    const double* d_info_matrices,
    const int* d_edge_var_i, const int* d_edge_var_j,
    const int* d_block_offsets,
    int num_edges, int num_free_poses,
    double* d_csr_values, double* d_b,
    const double* d_weights = nullptr);

void launchApplyPoseGraphDampingCSR(
    double* d_csr_values, const int* d_diag_offsets,
    int num_free_poses, int block_stride,
    double lambda, double min_diag);

void launchUpdatePoses(
    double* d_poses, const double* d_delta,
    const int* d_vertex_idx_map, int num_poses);

void launchComputePoseGraphCost(
    const double* d_residuals, const double* d_info_matrices,
    int num_edges, double* d_costs);

// Robust loss kernels
void launchComputeEdgeWeights(
    const double* d_residuals, const double* d_info_matrices,
    int num_edges, gpu::LossType loss_type, double loss_param,
    double* d_weights);

void launchComputePoseGraphRobustCost(
    const double* d_residuals, const double* d_info_matrices,
    int num_edges, gpu::LossType loss_type, double loss_param,
    double* d_costs);

// PCG support kernels
void launchSymmetricSpMV(
    const int* d_row_ptr, const int* d_col_ind, const double* d_values,
    const double* d_x, double* d_y, int num_rows);

void launchExtractAndInvertDiagBlocks(
    const double* d_csr_values, const int* d_diag_offsets,
    int num_free_poses, double* d_inv_blocks);

void launchApplyBlockPreconditioner(
    const double* d_inv_blocks, const double* d_r,
    int num_free_poses, double* d_z);

void launchAxpy(double alpha, const double* d_x, double* d_y, int n);

void launchXpby(const double* d_x, double beta, double* d_y, int n);

void launchNegate(const double* d_x, double* d_y, int n);

}  // namespace pg_gpu
}  // namespace solver
}  // namespace backend
}  // namespace substral
