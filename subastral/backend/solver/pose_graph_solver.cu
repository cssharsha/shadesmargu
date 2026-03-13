#include "subastral/backend/solver/pose_graph_solver.cuh"

#include <cusolverSp.h>
#include <cusparse.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/functional.h>
#include <thrust/reduce.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <vector>

#include "subastral/backend/solver/pose_graph_kernels.cuh"

namespace substral {
namespace backend {
namespace solver {

// =============================================================================
// CSR Sparsity Pattern Builder — Lower Triangle Only
// =============================================================================
//
// DESIGN NOTE: Why lower triangle, and how this differs from BA's Schur
// complement.
//
// In Bundle Adjustment, the Hessian has a special bipartite structure:
//
//   H = | U   W |     U = camera-camera (6×6 blocks)
//       | W^T V |     V = point-point (3×3 blocks, block-diagonal)
//                      W = camera-point cross terms
//
// Because V is block-diagonal, we can cheaply eliminate the points via the
// Schur complement:  S = U - W V^{-1} W^T,  then solve for cameras first
// and back-substitute for points. This is the standard BA trick.
//
// In a PURE POSE-GRAPH (pose-pose edges only, no landmarks), every variable
// is a pose (6 DOF) and every edge connects two poses. There is no bipartite
// structure, no block-diagonal sub-matrix, and no Schur complement to
// exploit. The Hessian is just a single sparse symmetric matrix H.
//
// Since H is symmetric positive-definite (with LM damping), we solve via
// sparse Cholesky factorization: H = L L^T. cuSOLVER's
// cusolverSpDcsrlsvchol with reorder=0 reads ONLY the lower triangle of
// the CSR data (treating it as H = L + D + L^T). So we store only the
// lower triangle to halve memory and nnz.
//
// FUTURE: When landmarks are added to the pose-graph (pose-landmark edges),
// the Hessian WILL have the bipartite U/W/V structure and we should use
// the Schur complement to eliminate landmarks, just like BA does.
// The current lower-triangle sparse Cholesky approach would still work
// but would be less efficient than Schur elimination for that case.
//
// Also computes:
//   - block_offsets: for each edge × 4 blocks × 6 rows, the CSR value
//     index where the row starts (used by the accumulation kernel)
//   - diag_offsets: for each free pose × 6 rows, the CSR value index
//     for the diagonal block row start (used by the damping kernel)
//
struct CSRPattern {
  int num_rows;
  int nnz;
  std::vector<int> row_ptr;   // [num_rows + 1]
  std::vector<int> col_ind;   // [nnz]

  // Per-edge: 4 blocks × 6 rows = 24 ints per edge
  // block_offsets[(k * 4 + b) * 6 + r] = CSR value index for row r,
  //   column 0 of block b of edge k. -1 if skipped.
  std::vector<int> block_offsets;

  // Per-free-pose: 6 ints
  // diag_offsets[p * 6 + r] = CSR value index for row r, column 0
  //   of the diagonal block of free pose p.
  std::vector<int> diag_offsets;

  // Per-edge: var indices (may be -1 for fixed)
  std::vector<int> edge_var_i;
  std::vector<int> edge_var_j;
};

static CSRPattern buildCSRPattern(
    const PoseGraphProblem& problem,
    const std::map<int, int>& id_to_pose_idx,
    const std::vector<int>& vertex_idx_map) {

  int num_free = problem.get_num_free_params() / 6;
  int num_rows = num_free * 6;
  int num_edges = problem.get_num_edges();

  // Step 1: Determine which block positions exist (LOWER TRIANGLE ONLY).
  // block_cols[var_a] = sorted set of var_b <= var_a such that block exists.
  std::vector<std::set<int>> block_cols(num_free);

  // Always add diagonal blocks
  for (int p = 0; p < num_free; ++p) {
    block_cols[p].insert(p);
  }

  // Per-edge variable indices
  std::vector<int> edge_var_i_vec(num_edges);
  std::vector<int> edge_var_j_vec(num_edges);

  for (int k = 0; k < num_edges; ++k) {
    int from_id = problem.edges[k]->get_from_id();
    int to_id = problem.edges[k]->get_to_id();

    int pose_idx_i = id_to_pose_idx.at(from_id);
    int pose_idx_j = id_to_pose_idx.at(to_id);
    int var_i = vertex_idx_map[pose_idx_i];
    int var_j = vertex_idx_map[pose_idx_j];

    edge_var_i_vec[k] = var_i;
    edge_var_j_vec[k] = var_j;

    if (var_i >= 0 && var_j >= 0) {
      // Lower triangle only: block (max, min)
      int lo = std::min(var_i, var_j);
      int hi = std::max(var_i, var_j);
      block_cols[hi].insert(lo);  // row=hi, col=lo (lower triangle)
    }
  }

  // Step 2: Expand to scalar CSR (lower triangle at scalar level).
  // For block row var_a, scalar rows are var_a*6+r for r=0..5.
  // For each scalar row, include:
  //   - All columns from off-diagonal blocks (var_b < var_a): full 6 cols
  //   - Diagonal block (var_b == var_a): only columns <= scalar_row
  //     (i.e., columns var_a*6+0 .. var_a*6+r)
  //
  // This gives us the scalar lower triangle.

  std::vector<int> row_ptr(num_rows + 1, 0);
  std::vector<int> col_ind_vec;

  for (int var_a = 0; var_a < num_free; ++var_a) {
    for (int r = 0; r < 6; ++r) {
      int scalar_row = var_a * 6 + r;
      int count = 0;
      for (int var_b : block_cols[var_a]) {
        if (var_b < var_a) {
          count += 6;  // full off-diagonal block
        } else {
          // Diagonal block: columns var_a*6+0 .. var_a*6+r
          count += r + 1;
        }
      }
      row_ptr[scalar_row + 1] = count;
    }
  }

  // Prefix sum
  for (int i = 0; i < num_rows; ++i) {
    row_ptr[i + 1] += row_ptr[i];
  }
  int nnz = row_ptr[num_rows];
  col_ind_vec.resize(nnz);

  // Fill col_ind
  for (int var_a = 0; var_a < num_free; ++var_a) {
    for (int r = 0; r < 6; ++r) {
      int scalar_row = var_a * 6 + r;
      int offset = row_ptr[scalar_row];
      int idx = 0;
      for (int var_b : block_cols[var_a]) {
        if (var_b < var_a) {
          for (int c = 0; c < 6; ++c) {
            col_ind_vec[offset + idx++] = var_b * 6 + c;
          }
        } else {
          // Diagonal: only cols 0..r
          for (int c = 0; c <= r; ++c) {
            col_ind_vec[offset + idx++] = var_a * 6 + c;
          }
        }
      }
    }
  }

  // Step 3: Build block_offsets lookup.
  // For each block (var_a, var_b) in the lower triangle, and each row r
  // of that block, find the CSR value index of the first column.
  //
  // For off-diagonal blocks (var_b < var_a): the 6 columns of the block
  // are contiguous in the CSR row.
  //
  // For diagonal blocks (var_b == var_a): row r of the block has columns
  // 0..r, so the "first column" offset is valid but only r+1 entries exist.

  // Precompute block positions within each row's column list
  std::vector<std::map<int, int>> block_col_start(num_free);
  for (int var_a = 0; var_a < num_free; ++var_a) {
    int pos = 0;
    for (int var_b : block_cols[var_a]) {
      block_col_start[var_a][var_b] = pos;
      if (var_b < var_a) {
        pos += 6;
      } else {
        // Diagonal: varies per row, but the starting position is the same
        // for all rows (it's the last block). We'll handle per-row below.
        pos = -1;  // sentinel, handled specially
        break;
      }
    }
  }

  auto getBlockRowOffset = [&](int var_a, int var_b, int r) -> int {
    if (var_a < 0 || var_b < 0) return -1;
    // Only lower triangle: var_a >= var_b
    if (var_a < var_b) return -1;

    auto it = block_col_start[var_a].find(var_b);
    if (it == block_col_start[var_a].end()) return -1;

    int scalar_row = var_a * 6 + r;

    if (var_b < var_a) {
      // Off-diagonal: block columns are at fixed position
      return row_ptr[scalar_row] + it->second;
    } else {
      // Diagonal: the diagonal block is always last in the sorted set.
      // Its start position in row r = total off-diagonal columns count.
      int off_diag_cols = 0;
      for (int vb : block_cols[var_a]) {
        if (vb < var_a) off_diag_cols += 6;
      }
      return row_ptr[scalar_row] + off_diag_cols;
    }
  };

  // For each edge, compute offsets for the 4 blocks.
  // block 0 = (ii): row=var_i, col=var_i → lower triangle only if diagonal
  // block 1 = (ij): row=var_i, col=var_j → lower triangle if var_i >= var_j
  // block 2 = (ji): row=var_j, col=var_i → lower triangle if var_j >= var_i
  // block 3 = (jj): row=var_j, col=var_j → lower triangle only if diagonal
  //
  // For the kernel, we only write to lower-triangle positions. The kernel
  // needs to know which of the 4 blocks are in the lower triangle.

  std::vector<int> block_offsets_vec(num_edges * 4 * 6);
  for (int k = 0; k < num_edges; ++k) {
    int var_i = edge_var_i_vec[k];
    int var_j = edge_var_j_vec[k];

    for (int r = 0; r < 6; ++r) {
      // block 0 = (ii): always in lower triangle (diagonal)
      block_offsets_vec[(k * 4 + 0) * 6 + r] = getBlockRowOffset(var_i, var_i, r);
      // block 1 = (ij): in lower triangle if var_i >= var_j
      block_offsets_vec[(k * 4 + 1) * 6 + r] =
          (var_i >= var_j) ? getBlockRowOffset(var_i, var_j, r) : -1;
      // block 2 = (ji): in lower triangle if var_j >= var_i
      block_offsets_vec[(k * 4 + 2) * 6 + r] =
          (var_j >= var_i) ? getBlockRowOffset(var_j, var_i, r) : -1;
      // block 3 = (jj): always in lower triangle (diagonal)
      block_offsets_vec[(k * 4 + 3) * 6 + r] = getBlockRowOffset(var_j, var_j, r);
    }
  }

  // Step 4: Build diag_offsets
  std::vector<int> diag_offsets_vec(num_free * 6);
  for (int p = 0; p < num_free; ++p) {
    for (int r = 0; r < 6; ++r) {
      diag_offsets_vec[p * 6 + r] = getBlockRowOffset(p, p, r);
    }
  }

  CSRPattern pattern;
  pattern.num_rows = num_rows;
  pattern.nnz = nnz;
  pattern.row_ptr = std::move(row_ptr);
  pattern.col_ind = std::move(col_ind_vec);
  pattern.block_offsets = std::move(block_offsets_vec);
  pattern.diag_offsets = std::move(diag_offsets_vec);
  pattern.edge_var_i = std::move(edge_var_i_vec);
  pattern.edge_var_j = std::move(edge_var_j_vec);

  return pattern;
}

// =============================================================================
// GPU Solver State
// =============================================================================

struct PGSolverState {
  int num_poses;
  int num_edges;
  int num_free;
  int total_dim;

  // Problem data (uploaded once)
  thrust::device_vector<double> d_poses;         // [num_poses * 7]
  thrust::device_vector<double> d_measurements;  // [num_edges * 7]
  thrust::device_vector<double> d_info_matrices;  // [num_edges * 36]
  thrust::device_vector<int> d_edge_from_idx;    // [num_edges] pose index
  thrust::device_vector<int> d_edge_to_idx;      // [num_edges] pose index
  thrust::device_vector<int> d_vertex_idx_map;   // [num_poses] var index (-1=fixed)

  // CSR pattern (uploaded once)
  thrust::device_vector<int> d_csr_row_ptr;      // [total_dim + 1]
  thrust::device_vector<int> d_csr_col_ind;      // [nnz]
  thrust::device_vector<int> d_block_offsets;    // [num_edges * 24]
  thrust::device_vector<int> d_diag_offsets;     // [num_free * 6]
  thrust::device_vector<int> d_edge_var_i;       // [num_edges]
  thrust::device_vector<int> d_edge_var_j;       // [num_edges]

  // Per-iteration workspace
  thrust::device_vector<double> d_residuals;     // [num_edges * 6]
  thrust::device_vector<double> d_J_i;           // [num_edges * 36]
  thrust::device_vector<double> d_J_j;           // [num_edges * 36]
  thrust::device_vector<double> d_csr_values;    // [nnz]
  thrust::device_vector<double> d_csr_values_damped; // [nnz]
  thrust::device_vector<double> d_b;             // [total_dim]
  thrust::device_vector<double> d_delta;         // [total_dim]
  thrust::device_vector<double> d_saved_poses;   // [num_poses * 7]
  thrust::device_vector<double> d_costs;         // [num_edges]

  // cuSOLVER sparse
  cusolverSpHandle_t cusolver_handle = nullptr;
  cusparseMatDescr_t mat_descr = nullptr;

  int nnz;

  void init(PoseGraphProblem& problem, const CSRPattern& pattern,
            const std::map<int, int>& id_to_pose_idx,
            const std::vector<int>& vertex_idx_map_vec) {
    num_poses = problem.get_num_poses();
    num_edges = problem.get_num_edges();
    num_free = problem.get_num_free_params() / 6;
    total_dim = num_free * 6;
    nnz = pattern.nnz;

    // Build contiguous pose array in index order
    auto sorted_ids = problem.sorted_vertex_ids();
    std::vector<double> h_poses(num_poses * 7);
    for (int i = 0; i < num_poses; ++i) {
      int id = sorted_ids[i];
      // Find the pose with this id
      const double* src = nullptr;
      for (const auto& p : problem.poses) {
        if (p->get_id() == id) { src = p->data(); break; }
      }
      for (int j = 0; j < 7; ++j) h_poses[i * 7 + j] = src[j];
    }

    // Build measurement and info arrays
    std::vector<double> h_measurements(num_edges * 7);
    std::vector<double> h_info(num_edges * 36);
    std::vector<int> h_edge_from(num_edges);
    std::vector<int> h_edge_to(num_edges);

    for (int k = 0; k < num_edges; ++k) {
      const auto& edge = problem.edges[k];
      const double* m = edge->measurement_data();
      const double* info = edge->info_data();
      for (int j = 0; j < 7; ++j) h_measurements[k * 7 + j] = m[j];
      for (int j = 0; j < 36; ++j) h_info[k * 36 + j] = info[j];
      h_edge_from[k] = id_to_pose_idx.at(edge->get_from_id());
      h_edge_to[k] = id_to_pose_idx.at(edge->get_to_id());
    }

    // Upload
    d_poses = h_poses;
    d_measurements = h_measurements;
    d_info_matrices = h_info;
    d_edge_from_idx = h_edge_from;
    d_edge_to_idx = h_edge_to;
    d_vertex_idx_map = vertex_idx_map_vec;

    d_csr_row_ptr = pattern.row_ptr;
    d_csr_col_ind = pattern.col_ind;
    d_block_offsets = pattern.block_offsets;
    d_diag_offsets = pattern.diag_offsets;
    d_edge_var_i = pattern.edge_var_i;
    d_edge_var_j = pattern.edge_var_j;

    // Allocate workspace
    d_residuals.resize(num_edges * 6);
    d_J_i.resize(num_edges * 36);
    d_J_j.resize(num_edges * 36);
    d_csr_values.resize(nnz, 0.0);
    d_csr_values_damped.resize(nnz, 0.0);
    d_b.resize(total_dim, 0.0);
    d_delta.resize(total_dim, 0.0);
    d_saved_poses.resize(num_poses * 7);
    d_costs.resize(num_edges);

    // cuSOLVER sparse Cholesky requires CUSPARSE_MATRIX_TYPE_GENERAL.
    // With reorder=0, it only reads the lower triangle of the CSR data.
    cusolverSpCreate(&cusolver_handle);
    cusparseCreateMatDescr(&mat_descr);
    cusparseSetMatType(mat_descr, CUSPARSE_MATRIX_TYPE_GENERAL);
    cusparseSetMatIndexBase(mat_descr, CUSPARSE_INDEX_BASE_ZERO);
  }

  void downloadPoses(PoseGraphProblem& problem,
                     const std::map<int, int>& id_to_pose_idx) {
    std::vector<double> h_poses(num_poses * 7);
    thrust::copy(d_poses.begin(), d_poses.end(), h_poses.begin());

    auto sorted_ids = problem.sorted_vertex_ids();
    for (int i = 0; i < num_poses; ++i) {
      int id = sorted_ids[i];
      for (auto& p : problem.poses) {
        if (p->get_id() == id) {
          double* dst = p->mutable_data();
          for (int j = 0; j < 7; ++j) dst[j] = h_poses[i * 7 + j];
          break;
        }
      }
    }
  }

  ~PGSolverState() {
    if (cusolver_handle) { cusolverSpDestroy(cusolver_handle); }
    if (mat_descr) { cusparseDestroyMatDescr(mat_descr); }
  }

  PGSolverState() = default;
  PGSolverState(const PGSolverState&) = delete;
  PGSolverState& operator=(const PGSolverState&) = delete;
};

// =============================================================================
// GPU Pose-Graph Solver
// =============================================================================

LMResult solvePoseGraph_GPU(PoseGraphProblem& problem,
                             const LMConfig& config,
                             PoseGraphCallback callback) {
  LMResult result;
  result.converged = false;
  result.iterations = 0;

  int num_poses = problem.get_num_poses();
  int num_edges = problem.get_num_edges();

  if (num_edges == 0) {
    result.initial_cost = 0;
    result.final_cost = 0;
    result.converged = true;
    result.termination_reason = "No edges";
    return result;
  }

  // Fix first vertex if not already set
  if (problem.fixed_vertex_id < 0 && !problem.poses.empty()) {
    problem.fixed_vertex_id = problem.poses[0]->get_id();
  }

  // Build index mappings
  auto sorted_ids = problem.sorted_vertex_ids();
  std::map<int, int> id_to_pose_idx;
  for (int i = 0; i < num_poses; ++i) {
    id_to_pose_idx[sorted_ids[i]] = i;
  }

  // vertex_idx_map[pose_idx] = var_idx (-1 for fixed)
  std::vector<int> vertex_idx_map(num_poses);
  int var_idx = 0;
  for (int i = 0; i < num_poses; ++i) {
    if (sorted_ids[i] == problem.fixed_vertex_id) {
      vertex_idx_map[i] = -1;
    } else {
      vertex_idx_map[i] = var_idx++;
    }
  }
  int num_free = var_idx;
  int total_dim = num_free * 6;

  if (config.verbose) {
    std::cout << "PG-GPU: " << num_poses << " poses (" << num_free
              << " free), " << num_edges << " edges, "
              << total_dim << " DOF" << std::endl;
  }

  // Build CSR pattern
  CSRPattern pattern = buildCSRPattern(problem, id_to_pose_idx,
                                        vertex_idx_map);

  if (config.verbose) {
    std::cout << "PG-GPU: CSR pattern: " << pattern.num_rows << " rows, "
              << pattern.nnz << " nnz" << std::endl;
  }

  // Initialize GPU state
  PGSolverState state;
  state.init(problem, pattern, id_to_pose_idx, vertex_idx_map);

  // Raw device pointers
  double* p_poses = thrust::raw_pointer_cast(state.d_poses.data());
  double* p_measurements = thrust::raw_pointer_cast(state.d_measurements.data());
  double* p_info = thrust::raw_pointer_cast(state.d_info_matrices.data());
  int* p_from = thrust::raw_pointer_cast(state.d_edge_from_idx.data());
  int* p_to = thrust::raw_pointer_cast(state.d_edge_to_idx.data());
  int* p_vtx_map = thrust::raw_pointer_cast(state.d_vertex_idx_map.data());
  int* p_row_ptr = thrust::raw_pointer_cast(state.d_csr_row_ptr.data());
  int* p_col_ind = thrust::raw_pointer_cast(state.d_csr_col_ind.data());
  int* p_block_off = thrust::raw_pointer_cast(state.d_block_offsets.data());
  int* p_diag_off = thrust::raw_pointer_cast(state.d_diag_offsets.data());
  int* p_edge_var_i = thrust::raw_pointer_cast(state.d_edge_var_i.data());
  int* p_edge_var_j = thrust::raw_pointer_cast(state.d_edge_var_j.data());
  double* p_residuals = thrust::raw_pointer_cast(state.d_residuals.data());
  double* p_Ji = thrust::raw_pointer_cast(state.d_J_i.data());
  double* p_Jj = thrust::raw_pointer_cast(state.d_J_j.data());
  double* p_csr_val = thrust::raw_pointer_cast(state.d_csr_values.data());
  double* p_csr_val_damped = thrust::raw_pointer_cast(state.d_csr_values_damped.data());
  double* p_b = thrust::raw_pointer_cast(state.d_b.data());
  double* p_delta = thrust::raw_pointer_cast(state.d_delta.data());
  double* p_saved = thrust::raw_pointer_cast(state.d_saved_poses.data());
  double* p_costs = thrust::raw_pointer_cast(state.d_costs.data());

  // Compute initial cost
  pg_gpu::launchComputePoseGraphResiduals(
      p_poses, p_measurements, p_from, p_to, num_edges, p_residuals);
  pg_gpu::launchComputePoseGraphCost(p_residuals, p_info, num_edges, p_costs);
  cudaDeviceSynchronize();

  double current_cost = 0.5 * thrust::reduce(
      state.d_costs.begin(), state.d_costs.end(), 0.0, thrust::plus<double>());

  result.initial_cost = current_cost;
  double lambda = config.initial_lambda;

  if (config.verbose) {
    std::cout << "PG-GPU: initial cost = " << current_cost << std::endl;
  }

  if (callback) callback(0, current_cost);

  // LM iteration loop
  for (int iter = 0; iter < config.max_iterations; ++iter) {
    result.iterations = iter + 1;

    if (current_cost <= 0.0) {
      result.converged = true;
      result.termination_reason = "Cost is zero";
      break;
    }

    // Step 1: Compute residuals
    pg_gpu::launchComputePoseGraphResiduals(
        p_poses, p_measurements, p_from, p_to, num_edges, p_residuals);

    // Step 2: Compute Jacobians
    pg_gpu::launchComputePoseGraphJacobians(
        p_poses, p_from, p_to, num_edges, p_residuals, p_Ji, p_Jj);

    // Step 3: Zero and accumulate Hessian + gradient
    cudaMemset(p_csr_val, 0, state.nnz * sizeof(double));
    cudaMemset(p_b, 0, total_dim * sizeof(double));

    pg_gpu::launchAccumulatePoseGraphHessianCSR(
        p_residuals, p_Ji, p_Jj, p_info,
        p_edge_var_i, p_edge_var_j, p_block_off,
        num_edges, num_free, p_csr_val, p_b);

    // Check gradient convergence
    {
      std::vector<double> h_b(total_dim);
      cudaMemcpy(h_b.data(), p_b, total_dim * sizeof(double),
                 cudaMemcpyDeviceToHost);
      double max_grad = 0.0;
      for (double v : h_b) max_grad = std::max(max_grad, std::abs(v));
      if (max_grad < config.gradient_tolerance) {
        result.converged = true;
        result.termination_reason = "Gradient below tolerance";
        break;
      }
    }

    // Save current poses
    cudaMemcpy(p_saved, p_poses, num_poses * 7 * sizeof(double),
               cudaMemcpyDeviceToDevice);

    bool step_accepted = false;

    for (int retry = 0; retry < 10; ++retry) {
      // Step 4: Copy CSR values and apply damping
      cudaMemcpy(p_csr_val_damped, p_csr_val, state.nnz * sizeof(double),
                 cudaMemcpyDeviceToDevice);

      pg_gpu::launchApplyPoseGraphDampingCSR(
          p_csr_val_damped, p_diag_off, num_free, 6, lambda, 1e-6);

      cudaDeviceSynchronize();

      // Step 5: Solve H_damped · δ = -b via cuSOLVER sparse Cholesky
      // Set up RHS: delta = -b
      {
        std::vector<double> h_b(total_dim);
        cudaMemcpy(h_b.data(), p_b, total_dim * sizeof(double),
                   cudaMemcpyDeviceToHost);
        std::vector<double> h_neg_b(total_dim);
        for (int i = 0; i < total_dim; ++i) h_neg_b[i] = -h_b[i];
        cudaMemcpy(p_delta, h_neg_b.data(), total_dim * sizeof(double),
                   cudaMemcpyHostToDevice);
      }

      int singularity = -1;
      cusolverStatus_t status = cusolverSpDcsrlsvchol(
          state.cusolver_handle,
          total_dim,
          state.nnz,
          state.mat_descr,
          p_csr_val_damped,
          p_row_ptr,
          p_col_ind,
          p_delta,   // RHS on input
          1e-14,     // tolerance
          0,         // reorder=0: only lower triangle needed (no full matrix)
          p_delta,   // solution on output
          &singularity);

      cudaDeviceSynchronize();

      if (status != CUSOLVER_STATUS_SUCCESS || singularity >= 0) {
        if (config.verbose) {
          std::cerr << "PG-GPU: sparse Cholesky failed (status=" << status
                    << ", singularity=" << singularity
                    << ", lambda=" << lambda << "), increasing lambda"
                    << std::endl;
        }
        lambda = std::min(lambda * config.lambda_factor, config.max_lambda);
        if (lambda >= config.max_lambda) {
          result.termination_reason = "Lambda exceeded maximum";
          break;
        }
        continue;
      }

      // Check step size
      {
        std::vector<double> h_delta(total_dim);
        cudaMemcpy(h_delta.data(), p_delta, total_dim * sizeof(double),
                   cudaMemcpyDeviceToHost);
        double step_norm = 0.0;
        for (double v : h_delta) step_norm += v * v;
        step_norm = std::sqrt(step_norm);

        if (step_norm < config.step_tolerance) {
          result.converged = true;
          result.termination_reason = "Step size below tolerance";
          break;
        }
      }

      // Step 6: Restore from saved, apply update
      cudaMemcpy(p_poses, p_saved, num_poses * 7 * sizeof(double),
                 cudaMemcpyDeviceToDevice);
      pg_gpu::launchUpdatePoses(p_poses, p_delta, p_vtx_map, num_poses);

      // Step 7: Evaluate new cost
      pg_gpu::launchComputePoseGraphResiduals(
          p_poses, p_measurements, p_from, p_to, num_edges, p_residuals);
      pg_gpu::launchComputePoseGraphCost(
          p_residuals, p_info, num_edges, p_costs);
      cudaDeviceSynchronize();

      double new_cost = 0.5 * thrust::reduce(
          state.d_costs.begin(), state.d_costs.end(),
          0.0, thrust::plus<double>());

      if (new_cost < current_cost) {
        double cost_change = current_cost - new_cost;

        if (config.verbose) {
          std::cout << "PG-GPU iter " << iter + 1 << ": cost " << current_cost
                    << " -> " << new_cost << " (delta=" << cost_change
                    << ", lambda=" << lambda << ")" << std::endl;
        }

        current_cost = new_cost;
        lambda = std::max(lambda / config.lambda_factor, config.min_lambda);
        step_accepted = true;

        if (callback) callback(iter + 1, current_cost);

        if (cost_change / (current_cost + 1e-12) < config.cost_tolerance) {
          result.converged = true;
          result.termination_reason = "Cost change below tolerance";
        }
        break;
      } else {
        // Reject: restore
        cudaMemcpy(p_poses, p_saved, num_poses * 7 * sizeof(double),
                   cudaMemcpyDeviceToDevice);
        lambda = std::min(lambda * config.lambda_factor, config.max_lambda);
        if (lambda >= config.max_lambda) {
          result.termination_reason = "Lambda exceeded maximum";
          break;
        }
      }
    }

    if (result.converged) break;
    if (!step_accepted) {
      result.termination_reason = "Failed to find a descent step";
      break;
    }
  }

  if (!result.converged && result.iterations >= config.max_iterations) {
    result.termination_reason = "Maximum iterations reached";
  }

  result.final_cost = current_cost;

  // Download final poses
  state.downloadPoses(problem, id_to_pose_idx);

  if (config.verbose) {
    std::cout << "PG-GPU: " << result.termination_reason << " after "
              << result.iterations << " iterations. Final cost = "
              << result.final_cost << std::endl;
  }

  return result;
}

}  // namespace solver
}  // namespace backend
}  // namespace substral
