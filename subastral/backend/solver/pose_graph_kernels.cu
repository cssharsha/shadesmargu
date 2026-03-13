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
    double* d_b,
    const double* __restrict__ d_weights) {

  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= num_edges) return;

  int var_i = d_edge_var_i[k];
  int var_j = d_edge_var_j[k];

  const double* Ji = &d_J_i[k * 36];
  const double* Jj = &d_J_j[k * 36];
  const double* e = &d_residuals[k * RES_DIM];
  const double* Omega = &d_info_matrices[k * INFO_DIM];

  // IRLS weight: w_k * Omega scales the information matrix for robust loss.
  // When d_weights is null (trivial loss), w_k = 1.0.
  double w_k = (d_weights != nullptr) ? d_weights[k] : 1.0;

  // Precompute w_k · Ω · J_i, w_k · Ω · J_j, w_k · Ω · e
  double wOmega[36];
  for (int i = 0; i < 36; ++i) wOmega[i] = w_k * Omega[i];

  double OJi[36], OJj[36], Oe[6];
  mat6x6_mul(wOmega, Ji, OJi);
  mat6x6_mul(wOmega, Jj, OJj);
  mat6x6_vec(wOmega, e, Oe);

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
// Kernel 13: Compute per-edge IRLS weight for robust loss
// =============================================================================
__global__ void computeEdgeWeightsKernel(
    const double* __restrict__ d_residuals,
    const double* __restrict__ d_info_matrices,
    int num_edges,
    gpu::LossType loss_type,
    double loss_param,
    double* d_weights) {

  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= num_edges) return;

  const double* e = &d_residuals[k * RES_DIM];
  const double* Omega = &d_info_matrices[k * INFO_DIM];

  // s = e^T * Omega * e (squared Mahalanobis distance)
  // The robust kernel acts on the full Mahalanobis distance so that
  // the loss-param threshold is in Mahalanobis units.
  double Oe[6];
  mat6x6_vec(Omega, e, Oe);
  double s = 0.0;
  for (int i = 0; i < 6; ++i) s += e[i] * Oe[i];

  // Guard against negative, NaN, or Inf
  if (s < 0.0 || isnan(s) || isinf(s)) {
    d_weights[k] = 1.0;
    return;
  }

  d_weights[k] = gpu::evalWeight(loss_type, s, loss_param);
}

// =============================================================================
// Kernel 14: Compute per-edge robust cost
// =============================================================================
__global__ void computePoseGraphRobustCostKernel(
    const double* __restrict__ d_residuals,
    const double* __restrict__ d_info_matrices,
    int num_edges,
    gpu::LossType loss_type,
    double loss_param,
    double* d_costs) {

  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= num_edges) return;

  const double* e = &d_residuals[k * RES_DIM];
  const double* Omega = &d_info_matrices[k * INFO_DIM];

  // Robust cost = rho(e^T * Omega * e)
  // Applied directly to the squared Mahalanobis distance.
  double Oe[6];
  mat6x6_vec(Omega, e, Oe);
  double s = 0.0;
  for (int i = 0; i < 6; ++i) s += e[i] * Oe[i];

  // Guard against negative, NaN, or Inf
  if (s < 0.0 || isnan(s) || isinf(s)) {
    d_costs[k] = 0.0;
    return;
  }

  d_costs[k] = gpu::evalRho(loss_type, s, loss_param);
}

// =============================================================================
// Kernel 7: Symmetric SpMV — y = A * x (lower-triangle CSR)
// =============================================================================
//
// Each thread handles one row. For each entry (i, j, val):
//   - j == i (diagonal): y[i] += val * x[i]
//   - j < i  (off-diag):  y[i] += val * x[j]  (direct)
//                          y[j] += val * x[i]  (symmetric, atomicAdd)
//
// y MUST be zeroed before calling.
//
__global__ void symmetricSpMVKernel(
    const int* __restrict__ row_ptr,
    const int* __restrict__ col_ind,
    const double* __restrict__ values,
    const double* __restrict__ x,
    double* y,
    int num_rows) {

  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_rows) return;

  int start = row_ptr[i];
  int end = row_ptr[i + 1];
  double xi = x[i];
  double yi = 0.0;

  for (int idx = start; idx < end; ++idx) {
    int j = col_ind[idx];
    double val = values[idx];

    if (j == i) {
      // Diagonal: count once
      yi += val * xi;
    } else {
      // Off-diagonal (j < i since lower triangle): symmetric contribution
      yi += val * x[j];
      atomicAdd(&y[j], val * xi);
    }
  }

  // Accumulate the direct contribution (may overlap with atomic writes
  // from other threads to y[i], so use atomicAdd)
  atomicAdd(&y[i], yi);
}

// =============================================================================
// Kernel 8: Extract and invert 6x6 diagonal blocks
// =============================================================================
//
// Each thread handles one free pose. Extracts the 6x6 diagonal block from
// the CSR values using diag_offsets, then inverts it via Gauss-Jordan
// elimination with partial pivoting.
//
// The diagonal block for pose p has a special structure in the lower-triangle
// CSR: row r of the block stores columns 0..r (lower triangle of the block).
// We reconstruct the full symmetric 6x6 block before inverting.
//
__global__ void extractAndInvertDiagBlocksKernel(
    const double* __restrict__ csr_values,
    const int* __restrict__ diag_offsets,
    int num_free_poses,
    double* inv_blocks) {

  int p = blockIdx.x * blockDim.x + threadIdx.x;
  if (p >= num_free_poses) return;

  // Extract the full symmetric 6x6 block from lower-triangle CSR
  double A[36];  // row-major 6x6

  for (int r = 0; r < 6; ++r) {
    int off = diag_offsets[p * 6 + r];
    // Lower triangle: columns 0..r are stored at csr_values[off + c]
    for (int c = 0; c <= r; ++c) {
      double val = csr_values[off + c];
      A[r * 6 + c] = val;
      A[c * 6 + r] = val;  // symmetric
    }
  }

  // Gauss-Jordan elimination with partial pivoting to compute A^{-1}
  // Augmented matrix [A | I] stored as A (in-place) and inv (identity)
  double inv[36];
  for (int i = 0; i < 36; ++i) inv[i] = 0.0;
  for (int i = 0; i < 6; ++i) inv[i * 6 + i] = 1.0;

  for (int col = 0; col < 6; ++col) {
    // Partial pivoting: find row with largest absolute value in column
    int pivot_row = col;
    double max_val = fabs(A[col * 6 + col]);
    for (int row = col + 1; row < 6; ++row) {
      double val = fabs(A[row * 6 + col]);
      if (val > max_val) {
        max_val = val;
        pivot_row = row;
      }
    }

    // Swap rows if needed
    if (pivot_row != col) {
      for (int k = 0; k < 6; ++k) {
        double tmp = A[col * 6 + k];
        A[col * 6 + k] = A[pivot_row * 6 + k];
        A[pivot_row * 6 + k] = tmp;

        tmp = inv[col * 6 + k];
        inv[col * 6 + k] = inv[pivot_row * 6 + k];
        inv[pivot_row * 6 + k] = tmp;
      }
    }

    // Scale pivot row
    double pivot = A[col * 6 + col];
    if (fabs(pivot) < 1e-30) {
      // Singular block — set inverse to identity as fallback
      for (int i = 0; i < 36; ++i) inv[i] = 0.0;
      for (int i = 0; i < 6; ++i) inv[i * 6 + i] = 1.0;
      break;
    }

    double inv_pivot = 1.0 / pivot;
    for (int k = 0; k < 6; ++k) {
      A[col * 6 + k] *= inv_pivot;
      inv[col * 6 + k] *= inv_pivot;
    }

    // Eliminate column in all other rows
    for (int row = 0; row < 6; ++row) {
      if (row == col) continue;
      double factor = A[row * 6 + col];
      for (int k = 0; k < 6; ++k) {
        A[row * 6 + k] -= factor * A[col * 6 + k];
        inv[row * 6 + k] -= factor * inv[col * 6 + k];
      }
    }
  }

  // Store result
  for (int i = 0; i < 36; ++i) {
    inv_blocks[p * 36 + i] = inv[i];
  }
}

// =============================================================================
// Kernel 9: Apply block-diagonal preconditioner — z = M^{-1} * r
// =============================================================================
__global__ void applyBlockPreconditionerKernel(
    const double* __restrict__ inv_blocks,
    const double* __restrict__ r_vec,
    int num_free_poses,
    double* z) {

  int p = blockIdx.x * blockDim.x + threadIdx.x;
  if (p >= num_free_poses) return;

  const double* M_inv = &inv_blocks[p * 36];
  const double* r = &r_vec[p * 6];
  double* zp = &z[p * 6];

  for (int i = 0; i < 6; ++i) {
    double sum = 0.0;
    for (int j = 0; j < 6; ++j) {
      sum += M_inv[i * 6 + j] * r[j];
    }
    zp[i] = sum;
  }
}

// =============================================================================
// Kernel 10: axpy — y = alpha * x + y
// =============================================================================
__global__ void axpyKernel(double alpha, const double* __restrict__ x,
                           double* y, int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  y[i] += alpha * x[i];
}

// =============================================================================
// Kernel 11: xpby — y = x + beta * y
// =============================================================================
__global__ void xpbyKernel(const double* __restrict__ x, double beta,
                           double* y, int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  y[i] = x[i] + beta * y[i];
}

// =============================================================================
// Kernel 12: negate — y = -x
// =============================================================================
__global__ void negateKernel(const double* __restrict__ x, double* y, int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  y[i] = -x[i];
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
    double* d_csr_values, double* d_b,
    const double* d_weights) {
  accumulatePoseGraphHessianCSRKernel<<<gridSize(num_edges), BLOCK_SIZE>>>(
      d_residuals, d_J_i, d_J_j, d_info_matrices,
      d_edge_var_i, d_edge_var_j, d_block_offsets,
      num_edges, num_free_poses, d_csr_values, d_b, d_weights);
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

void launchComputeEdgeWeights(
    const double* d_residuals, const double* d_info_matrices,
    int num_edges, gpu::LossType loss_type, double loss_param,
    double* d_weights) {
  computeEdgeWeightsKernel<<<gridSize(num_edges), BLOCK_SIZE>>>(
      d_residuals, d_info_matrices, num_edges, loss_type, loss_param,
      d_weights);
}

void launchComputePoseGraphRobustCost(
    const double* d_residuals, const double* d_info_matrices,
    int num_edges, gpu::LossType loss_type, double loss_param,
    double* d_costs) {
  computePoseGraphRobustCostKernel<<<gridSize(num_edges), BLOCK_SIZE>>>(
      d_residuals, d_info_matrices, num_edges, loss_type, loss_param,
      d_costs);
}

void launchSymmetricSpMV(
    const int* d_row_ptr, const int* d_col_ind, const double* d_values,
    const double* d_x, double* d_y, int num_rows) {
  symmetricSpMVKernel<<<gridSize(num_rows), BLOCK_SIZE>>>(
      d_row_ptr, d_col_ind, d_values, d_x, d_y, num_rows);
}

void launchExtractAndInvertDiagBlocks(
    const double* d_csr_values, const int* d_diag_offsets,
    int num_free_poses, double* d_inv_blocks) {
  extractAndInvertDiagBlocksKernel<<<gridSize(num_free_poses), BLOCK_SIZE>>>(
      d_csr_values, d_diag_offsets, num_free_poses, d_inv_blocks);
}

void launchApplyBlockPreconditioner(
    const double* d_inv_blocks, const double* d_r,
    int num_free_poses, double* d_z) {
  applyBlockPreconditionerKernel<<<gridSize(num_free_poses), BLOCK_SIZE>>>(
      d_inv_blocks, d_r, num_free_poses, d_z);
}

void launchAxpy(double alpha, const double* d_x, double* d_y, int n) {
  axpyKernel<<<gridSize(n), BLOCK_SIZE>>>(alpha, d_x, d_y, n);
}

void launchXpby(const double* d_x, double beta, double* d_y, int n) {
  xpbyKernel<<<gridSize(n), BLOCK_SIZE>>>(d_x, beta, d_y, n);
}

void launchNegate(const double* d_x, double* d_y, int n) {
  negateKernel<<<gridSize(n), BLOCK_SIZE>>>(d_x, d_y, n);
}

}  // namespace pg_gpu
}  // namespace solver
}  // namespace backend
}  // namespace substral
