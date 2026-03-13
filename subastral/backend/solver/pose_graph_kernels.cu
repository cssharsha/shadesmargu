#include "subastral/backend/solver/pose_graph_kernels.cuh"

namespace substral {
namespace backend {
namespace solver {
namespace pg_gpu {

static constexpr int BLOCK_SIZE = 256;

static inline int gridSize(int n) {
  return (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
}

// =============================================================================
// Device helper: SE(3) left Jacobian inverse (block-diagonal approximation)
// =============================================================================
//
//   J_l^{-1}(ξ) ≈ | J_l^{-1}_{SO(3)}(ω)    0                    |
//                  |       0                  J_l^{-1}_{SO(3)}(ω) |
//
__device__ void leftJacobianInverseSE3_approx(const double* xi,
                                               double* Jl_inv) {
  for (int i = 0; i < 36; ++i) Jl_inv[i] = 0.0;

  double Jl_inv_so3[9];
  lie::device::leftJacobianInverseSO3(xi, Jl_inv_so3);

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      Jl_inv[i * 6 + j] = Jl_inv_so3[i * 3 + j];
      Jl_inv[(i + 3) * 6 + (j + 3)] = Jl_inv_so3[i * 3 + j];
    }
}

// =============================================================================
// Kernel 1: Compute per-edge residuals
// =============================================================================
__global__ void computePoseGraphResidualsKernel(
    const double* __restrict__ d_poses,
    const double* __restrict__ d_measurements,
    const int* __restrict__ d_edge_from_idx,
    const int* __restrict__ d_edge_to_idx,
    int num_edges,
    double* d_residuals) {

  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= num_edges) return;

  int idx_i = d_edge_from_idx[k];
  int idx_j = d_edge_to_idx[k];

  const double* pose_i = &d_poses[idx_i * POSE_DIM];
  const double* pose_j = &d_poses[idx_j * POSE_DIM];
  const double* meas = &d_measurements[k * POSE_DIM];

  double Ri[9], ti[3], Rj[9], tj[3], Rm[9], tm[3];
  poseToRT(pose_i, Ri, ti);
  poseToRT(pose_j, Rj, tj);
  poseToRT(meas, Rm, tm);

  // T̄_ij^{-1}
  double Rm_inv[9], tm_inv[3];
  lie::device::inverseSE3(Rm, tm, Rm_inv, tm_inv);

  // T_i^{-1}
  double Ri_inv[9], ti_inv[3];
  lie::device::inverseSE3(Ri, ti, Ri_inv, ti_inv);

  // T_i^{-1} · T_j
  double R_ij[9], t_ij[3];
  lie::device::composeSE3(Ri_inv, ti_inv, Rj, tj, R_ij, t_ij);

  // T̄_ij^{-1} · (T_i^{-1} · T_j)
  double R_err[9], t_err[3];
  lie::device::composeSE3(Rm_inv, tm_inv, R_ij, t_ij, R_err, t_err);

  // e = log(E)
  lie::device::logSE3(R_err, t_err, &d_residuals[k * RES_DIM]);
}

// =============================================================================
// Kernel 2: Compute per-edge Jacobians
// =============================================================================
//
// For left perturbation T_i <- exp(δξ_i) · T_i:
//
//   E(ε_i) = T̄^{-1} · T_i^{-1} · exp(-ε_i) · T_j
//          = E_0 · exp(-Ad(T_j^{-1}) · ε_i)
//
//   E(ε_j) = T̄^{-1} · T_i^{-1} · exp(ε_j) · T_j
//          = E_0 · exp(Ad(T_j^{-1}) · ε_j)
//
// Using BCH: log(E_0 · exp(δ)) ≈ e_0 + J_r^{-1}(e_0) · δ
//
//   ∂e/∂ξ_i = -J_r^{-1}(e) · Ad(T_j^{-1})
//   ∂e/∂ξ_j =  J_r^{-1}(e) · Ad(T_j^{-1})
//
// Note: J_r^{-1}(ξ) = J_l^{-1}(-ξ)
//
__global__ void computePoseGraphJacobiansKernel(
    const double* __restrict__ d_poses,
    const int* __restrict__ d_edge_from_idx,
    const int* __restrict__ d_edge_to_idx,
    int num_edges,
    const double* __restrict__ d_residuals,
    double* d_J_i,
    double* d_J_j) {

  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= num_edges) return;

  int idx_j = d_edge_to_idx[k];
  const double* pose_j = &d_poses[idx_j * POSE_DIM];

  double Rj[9], tj[3];
  poseToRT(pose_j, Rj, tj);

  // T_j^{-1}
  double Rj_inv[9], tj_inv[3];
  lie::device::inverseSE3(Rj, tj, Rj_inv, tj_inv);

  // Ad(T_j^{-1})
  double Ad_j_inv[36];
  adjointSE3(Rj_inv, tj_inv, Ad_j_inv);

  // J_r^{-1}(e_k) = J_l^{-1}(-e_k)
  const double* e_k = &d_residuals[k * RES_DIM];
  double neg_e[6];
  for (int i = 0; i < 6; ++i) neg_e[i] = -e_k[i];
  double Jr_inv[36];
  leftJacobianInverseSE3_approx(neg_e, Jr_inv);

  // ∂e/∂ξ_j = J_r^{-1}(e) · Ad(T_j^{-1})
  double* Jj = &d_J_j[k * 36];
  mat6x6_mul(Jr_inv, Ad_j_inv, Jj);

  // ∂e/∂ξ_i = -J_r^{-1}(e) · Ad(T_j^{-1}) = -Jj
  double* Ji = &d_J_i[k * 36];
  for (int i = 0; i < 36; ++i) Ji[i] = -Jj[i];
}

// =============================================================================
// Kernel 3: Accumulate Hessian (lower triangle) into CSR values + gradient
// =============================================================================
//
// d_block_offsets[(k * 4 + b) * 6 + r] = CSR value index for row r,
//   column 0 of block b of edge k. -1 if block is skipped.
//
// For diagonal blocks (ii, jj): row r only stores columns 0..r
//   (lower-triangle scalar entries). So we write c <= r only.
//
// For off-diagonal blocks (ij, ji): only one of them is in the lower
//   triangle. All 6 columns are stored. The block_offsets for the
//   upper-triangle block are set to -1 by the CPU builder.
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
    double* d_b) {

  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= num_edges) return;

  int var_i = d_edge_var_i[k];
  int var_j = d_edge_var_j[k];

  const double* Ji = &d_J_i[k * 36];
  const double* Jj = &d_J_j[k * 36];
  const double* e = &d_residuals[k * RES_DIM];
  const double* Omega = &d_info_matrices[k * INFO_DIM];

  // Precompute Ω · J_i, Ω · J_j, Ω · e
  double OJi[36], OJj[36], Oe[6];
  mat6x6_mul(Omega, Ji, OJi);
  mat6x6_mul(Omega, Jj, OJj);
  mat6x6_vec(Omega, e, Oe);

  // Block 0 = (ii): diagonal, lower triangle → c <= r
  if (var_i >= 0) {
    for (int r = 0; r < 6; ++r) {
      int row_off = d_block_offsets[(k * 4 + 0) * 6 + r];
      if (row_off < 0) continue;
      for (int c = 0; c <= r; ++c) {
        double val = 0.0;
        for (int m = 0; m < 6; ++m)
          val += Ji[m * 6 + r] * OJi[m * 6 + c];
        atomicAdd(&d_csr_values[row_off + c], val);
      }
      // Gradient
      double bval = 0.0;
      for (int m = 0; m < 6; ++m)
        bval += Ji[m * 6 + r] * Oe[m];
      atomicAdd(&d_b[var_i * TANGENT_DIM + r], bval);
    }
  }

  // Block 3 = (jj): diagonal, lower triangle → c <= r
  if (var_j >= 0) {
    for (int r = 0; r < 6; ++r) {
      int row_off = d_block_offsets[(k * 4 + 3) * 6 + r];
      if (row_off < 0) continue;
      for (int c = 0; c <= r; ++c) {
        double val = 0.0;
        for (int m = 0; m < 6; ++m)
          val += Jj[m * 6 + r] * OJj[m * 6 + c];
        atomicAdd(&d_csr_values[row_off + c], val);
      }
      double bval = 0.0;
      for (int m = 0; m < 6; ++m)
        bval += Jj[m * 6 + r] * Oe[m];
      atomicAdd(&d_b[var_j * TANGENT_DIM + r], bval);
    }
  }

  // Block 1 = (ij): off-diagonal, in lower triangle if var_i >= var_j
  // All 6 columns stored (off-diagonal blocks are fully stored)
  if (var_i >= 0 && var_j >= 0) {
    for (int r = 0; r < 6; ++r) {
      int row_off = d_block_offsets[(k * 4 + 1) * 6 + r];
      if (row_off < 0) continue;
      for (int c = 0; c < 6; ++c) {
        double val = 0.0;
        for (int m = 0; m < 6; ++m)
          val += Ji[m * 6 + r] * OJj[m * 6 + c];
        atomicAdd(&d_csr_values[row_off + c], val);
      }
    }
  }

  // Block 2 = (ji): off-diagonal, in lower triangle if var_j >= var_i
  if (var_i >= 0 && var_j >= 0) {
    for (int r = 0; r < 6; ++r) {
      int row_off = d_block_offsets[(k * 4 + 2) * 6 + r];
      if (row_off < 0) continue;
      for (int c = 0; c < 6; ++c) {
        double val = 0.0;
        for (int m = 0; m < 6; ++m)
          val += Jj[m * 6 + r] * OJi[m * 6 + c];
        atomicAdd(&d_csr_values[row_off + c], val);
      }
    }
  }
}

// =============================================================================
// Kernel 4: Apply LM damping to CSR diagonal blocks
// =============================================================================
__global__ void applyPoseGraphDampingCSRKernel(
    double* d_csr_values,
    const int* __restrict__ d_diag_offsets,
    int num_free_poses,
    int block_stride,
    double lambda,
    double min_diag) {

  int p = blockIdx.x * blockDim.x + threadIdx.x;
  if (p >= num_free_poses) return;

  // d_diag_offsets[p * 6 + r] = CSR value index for row r, col 0 of
  // the diagonal block of pose p.
  // The diagonal element within row r is at column r (offset + r).
  for (int r = 0; r < 6; ++r) {
    int off = d_diag_offsets[p * 6 + r];
    double diag_val = d_csr_values[off + r];
    d_csr_values[off + r] += lambda * fmax(diag_val, min_diag);
  }
}

// =============================================================================
// Kernel 5: Update poses
// =============================================================================
__global__ void updatePosesKernel(
    double* d_poses,
    const double* __restrict__ d_delta,
    const int* __restrict__ d_vertex_idx_map,
    int num_poses) {

  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_poses) return;

  int var_idx = d_vertex_idx_map[i];
  if (var_idx < 0) return;

  const double* xi = &d_delta[var_idx * TANGENT_DIM];
  double* pose = &d_poses[i * POSE_DIM];

  double R_old[9], t_old[3];
  poseToRT(pose, R_old, t_old);

  double R_delta[9], t_delta[3];
  lie::device::expSE3(xi, R_delta, t_delta);

  double R_new[9], t_new[3];
  lie::device::composeSE3(R_delta, t_delta, R_old, t_old, R_new, t_new);

  rtToPose(R_new, t_new, pose);
}

// =============================================================================
// Kernel 6: Compute per-edge cost
// =============================================================================
__global__ void computePoseGraphCostKernel(
    const double* __restrict__ d_residuals,
    const double* __restrict__ d_info_matrices,
    int num_edges,
    double* d_costs) {

  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= num_edges) return;

  const double* e = &d_residuals[k * RES_DIM];
  const double* Omega = &d_info_matrices[k * INFO_DIM];

  double Oe[6];
  mat6x6_vec(Omega, e, Oe);

  double cost = 0.0;
  for (int i = 0; i < 6; ++i) cost += e[i] * Oe[i];
  d_costs[k] = cost;
}

// =============================================================================
// Host wrappers
// =============================================================================

void launchComputePoseGraphResiduals(
    const double* d_poses, const double* d_measurements,
    const int* d_edge_from_idx, const int* d_edge_to_idx,
    int num_edges, double* d_residuals) {
  computePoseGraphResidualsKernel<<<gridSize(num_edges), BLOCK_SIZE>>>(
      d_poses, d_measurements, d_edge_from_idx, d_edge_to_idx,
      num_edges, d_residuals);
}

void launchComputePoseGraphJacobians(
    const double* d_poses,
    const int* d_edge_from_idx, const int* d_edge_to_idx,
    int num_edges, const double* d_residuals,
    double* d_J_i, double* d_J_j) {
  computePoseGraphJacobiansKernel<<<gridSize(num_edges), BLOCK_SIZE>>>(
      d_poses, d_edge_from_idx, d_edge_to_idx,
      num_edges, d_residuals, d_J_i, d_J_j);
}

void launchAccumulatePoseGraphHessianCSR(
    const double* d_residuals, const double* d_J_i, const double* d_J_j,
    const double* d_info_matrices,
    const int* d_edge_var_i, const int* d_edge_var_j,
    const int* d_block_offsets,
    int num_edges, int num_free_poses,
    double* d_csr_values, double* d_b) {
  accumulatePoseGraphHessianCSRKernel<<<gridSize(num_edges), BLOCK_SIZE>>>(
      d_residuals, d_J_i, d_J_j, d_info_matrices,
      d_edge_var_i, d_edge_var_j, d_block_offsets,
      num_edges, num_free_poses, d_csr_values, d_b);
}

void launchApplyPoseGraphDampingCSR(
    double* d_csr_values, const int* d_diag_offsets,
    int num_free_poses, int block_stride,
    double lambda, double min_diag) {
  applyPoseGraphDampingCSRKernel<<<gridSize(num_free_poses), BLOCK_SIZE>>>(
      d_csr_values, d_diag_offsets, num_free_poses, block_stride,
      lambda, min_diag);
}

void launchUpdatePoses(
    double* d_poses, const double* d_delta,
    const int* d_vertex_idx_map, int num_poses) {
  updatePosesKernel<<<gridSize(num_poses), BLOCK_SIZE>>>(
      d_poses, d_delta, d_vertex_idx_map, num_poses);
}

void launchComputePoseGraphCost(
    const double* d_residuals, const double* d_info_matrices,
    int num_edges, double* d_costs) {
  computePoseGraphCostKernel<<<gridSize(num_edges), BLOCK_SIZE>>>(
      d_residuals, d_info_matrices, num_edges, d_costs);
}

}  // namespace pg_gpu
}  // namespace solver
}  // namespace backend
}  // namespace substral
