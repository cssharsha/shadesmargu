#include "subastral/backend/solver/lm_solver_gpu.cuh"

#include <cusolverDn.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/functional.h>
#include <thrust/reduce.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

#include "subastral/backend/ops/jacobians.cuh"
#include "subastral/backend/solver/gpu_solver_kernels.cuh"

namespace substral {
namespace backend {
namespace solver {

// =============================================================================
// GPUSolverState: owns all device memory for the LM loop
// =============================================================================
//
// Allocated once at solver start. Freed when the struct goes out of scope
// (thrust::device_vector handles deallocation).
//
// The problem data (cameras, points, observations, indices) is uploaded
// from the host BAProblem. Parameters (cameras, points) are modified
// in-place on the device during optimization. At the end, the final
// parameters are downloaded back to the host BAProblem.
//
// The dense Schur complement solve uses cuSOLVER's Cholesky factorization
// (dpotrf + dpotrs) entirely on GPU, eliminating D2H/H2D transfers of
// the Schur matrix and solution vector.
//
// =============================================================================

struct GPUSolverState {
  int num_cameras;
  int num_points;
  int num_obs;
  int total_cam_dim;  // num_cameras * 9
  int total_pt_dim;   // num_points * 3

  // ---- Problem data (uploaded once) ----
  thrust::device_vector<double> d_cameras;       // [num_cameras * 9]
  thrust::device_vector<double> d_points;        // [num_points * 3]
  thrust::device_vector<double> d_observations;  // [num_obs * 2]
  thrust::device_vector<int> d_cam_indices;      // [num_obs]
  thrust::device_vector<int> d_pt_indices;       // [num_obs]

  // CSR structure for point -> observations mapping
  thrust::device_vector<int> d_point_obs_offsets;  // [num_points + 1]
  thrust::device_vector<int> d_point_obs_list;     // [num_obs]

  // ---- Per-iteration workspace ----
  thrust::device_vector<double> d_residuals;      // [num_obs * 2]
  thrust::device_vector<double> d_J_cameras;      // [num_obs * 18]
  thrust::device_vector<double> d_J_points;       // [num_obs * 6]

  thrust::device_vector<double> d_U;              // [num_cameras * 81]
  thrust::device_vector<double> d_V;              // [num_points * 9]
  thrust::device_vector<double> d_W;              // [num_obs * 27]
  thrust::device_vector<double> d_g_c;            // [num_cameras * 9]
  thrust::device_vector<double> d_g_p;            // [num_points * 3]

  // Damped copies of U and V for the inner LM retry loop.
  // LM damping is applied to these before Schur complement formation,
  // ensuring V_damped is always well-conditioned for inversion and the
  // resulting Schur complement is SPD.
  thrust::device_vector<double> d_U_damped;       // [num_cameras * 81]
  thrust::device_vector<double> d_V_damped;       // [num_points * 9]

  thrust::device_vector<double> d_S;              // [total_cam_dim * total_cam_dim]
  thrust::device_vector<double> d_rhs;            // [total_cam_dim]

  thrust::device_vector<double> d_delta_cameras;  // [total_cam_dim]
  thrust::device_vector<double> d_delta_points;   // [total_pt_dim]

  thrust::device_vector<double> d_saved_cameras;  // [num_cameras * 9]
  thrust::device_vector<double> d_saved_points;   // [num_points * 3]

  thrust::device_vector<double> d_sq_residuals;   // [num_obs]

  // ---- cuSOLVER dense Cholesky ----
  //
  // The damped Schur complement S + lambda*diag(S) is symmetric positive
  // definite (SPD) by construction — LM damping ensures this. We use
  // cuSOLVER's dense Cholesky factorization (dpotrf) followed by the
  // triangular solve (dpotrs) to solve (S_damped) * delta_c = -rhs
  // entirely on GPU.
  //
  // d_S_factored: working copy of S that gets damped and factorized
  //   in-place by dpotrf. We keep the original d_S intact for lambda
  //   retries in the inner loop.
  //
  // d_cusolver_workspace: scratch buffer required by cuSOLVER, sized
  //   via cusolverDnDpotrf_bufferSize at init time.
  //
  // d_cusolver_info: single int on device, set by dpotrf to indicate
  //   factorization status (0 = success, >0 = not SPD).
  //
  cusolverDnHandle_t cusolver_handle = nullptr;
  thrust::device_vector<double> d_S_factored;      // [total_cam_dim^2]
  thrust::device_vector<double> d_cusolver_workspace;
  thrust::device_vector<int> d_cusolver_info;       // [1]
  int cusolver_workspace_size = 0;

  // ---- Host buffers for Eigen fallback ----
  // Kept for the rare case where cuSOLVER Cholesky fails (matrix not SPD).
  // Eigen's LDLT handles indefinite matrices gracefully.
  std::vector<double> h_S;
  std::vector<double> h_rhs;
  std::vector<double> h_delta_cameras;

  void init(BAProblem& problem) {
    num_cameras = static_cast<int>(problem.cameras.size());
    num_points = static_cast<int>(problem.points.size());
    num_obs = static_cast<int>(problem.observations.size());
    total_cam_dim = num_cameras * gpu::CAM_DIM;
    total_pt_dim = num_points * gpu::PT_DIM;

    // Upload problem data
    d_cameras = problem.memory_map->observers;
    d_points = problem.memory_map->scene_points;
    d_observations = problem.memory_map->observations;
    d_cam_indices = problem.memory_map->observation_camera_indices;
    d_pt_indices = problem.memory_map->observation_point_indices;

    // Build CSR structure for point -> observations mapping
    // This is O(num_obs) on the CPU, done once.
    std::vector<int> point_obs_offsets(num_points + 1, 0);
    std::vector<int> point_obs_list(num_obs);

    // Count observations per point
    for (int k = 0; k < num_obs; ++k) {
      int pt_idx = problem.memory_map->observation_point_indices[k];
      point_obs_offsets[pt_idx + 1]++;
    }
    // Prefix sum
    for (int j = 0; j < num_points; ++j) {
      point_obs_offsets[j + 1] += point_obs_offsets[j];
    }
    // Fill observation list
    std::vector<int> counters(num_points, 0);
    for (int k = 0; k < num_obs; ++k) {
      int pt_idx = problem.memory_map->observation_point_indices[k];
      int pos = point_obs_offsets[pt_idx] + counters[pt_idx];
      point_obs_list[pos] = k;
      counters[pt_idx]++;
    }

    d_point_obs_offsets = point_obs_offsets;
    d_point_obs_list = point_obs_list;

    // Allocate workspace
    d_residuals.resize(num_obs * 2);
    d_J_cameras.resize(num_obs * 18);
    d_J_points.resize(num_obs * 6);

    d_U.resize(num_cameras * gpu::U_BLOCK);
    d_V.resize(num_points * gpu::V_BLOCK);
    d_W.resize(num_obs * gpu::W_BLOCK);
    d_g_c.resize(num_cameras * gpu::CAM_DIM);
    d_g_p.resize(num_points * gpu::PT_DIM);

    d_U_damped.resize(num_cameras * gpu::U_BLOCK);
    d_V_damped.resize(num_points * gpu::V_BLOCK);

    d_S.resize(total_cam_dim * total_cam_dim);
    d_rhs.resize(total_cam_dim);

    d_delta_cameras.resize(total_cam_dim);
    d_delta_points.resize(total_pt_dim);

    d_saved_cameras.resize(num_cameras * gpu::CAM_DIM);
    d_saved_points.resize(num_points * gpu::PT_DIM);

    d_sq_residuals.resize(num_obs);

    // ---- Initialize cuSOLVER ----
    cusolverDnCreate(&cusolver_handle);

    d_S_factored.resize(total_cam_dim * total_cam_dim);
    d_cusolver_info.resize(1);

    // Query workspace size for Cholesky factorization.
    // CUBLAS_FILL_MODE_UPPER: our S is row-major, which cuSOLVER (column-major)
    // sees as the transpose. For symmetric S, we use the upper triangle.
    cusolverDnDpotrf_bufferSize(
        cusolver_handle, CUBLAS_FILL_MODE_UPPER,
        total_cam_dim,
        thrust::raw_pointer_cast(d_S_factored.data()),
        total_cam_dim,
        &cusolver_workspace_size);
    d_cusolver_workspace.resize(cusolver_workspace_size);

    // Host buffers for Eigen fallback
    h_S.resize(total_cam_dim * total_cam_dim);
    h_rhs.resize(total_cam_dim);
    h_delta_cameras.resize(total_cam_dim);
  }

  // Download final parameters back to host BAProblem
  void downloadToHost(BAProblem& problem) {
    thrust::copy(d_cameras.begin(), d_cameras.end(),
                 problem.memory_map->observers.begin());
    thrust::copy(d_points.begin(), d_points.end(),
                 problem.memory_map->scene_points.begin());
  }

  ~GPUSolverState() {
    if (cusolver_handle) {
      cusolverDnDestroy(cusolver_handle);
      cusolver_handle = nullptr;
    }
  }

  // Non-copyable (owns CUDA resources)
  GPUSolverState() = default;
  GPUSolverState(const GPUSolverState&) = delete;
  GPUSolverState& operator=(const GPUSolverState&) = delete;
};

// =============================================================================
// GPU LM Solver Implementation
// =============================================================================

static constexpr int BLOCK_SIZE = 256;

static inline int gridSize(int n) {
  return (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
}

// =============================================================================
// Helper: negate a vector on GPU
// =============================================================================
//
// d_out[i] = -d_in[i]  for i in [0, n)
//
// Used to compute -rhs for the Cholesky solve without a host roundtrip.
//
__global__ void negateVectorKernel(const double* __restrict__ d_in,
                                   double* d_out, int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    d_out[i] = -d_in[i];
  }
}

LMResult solveLM_GPU(BAProblem& problem, const LMConfig& config) {
  LMResult result;
  result.converged = false;
  result.iterations = 0;

  int num_obs = static_cast<int>(problem.observations.size());
  if (num_obs == 0) {
    result.initial_cost = 0;
    result.final_cost = 0;
    result.converged = true;
    result.termination_reason = "No observations";
    return result;
  }

  // ---- Initialize GPU state ----
  GPUSolverState state;
  state.init(problem);

  int num_cameras = state.num_cameras;
  int num_points = state.num_points;
  int total_cam_dim = state.total_cam_dim;
  int total_pt_dim = state.total_pt_dim;

  // Raw device pointers for kernel launches
  double* p_cameras = thrust::raw_pointer_cast(state.d_cameras.data());
  double* p_points = thrust::raw_pointer_cast(state.d_points.data());
  double* p_observations = thrust::raw_pointer_cast(state.d_observations.data());
  int* p_cam_indices = thrust::raw_pointer_cast(state.d_cam_indices.data());
  int* p_pt_indices = thrust::raw_pointer_cast(state.d_pt_indices.data());
  int* p_point_obs_offsets = thrust::raw_pointer_cast(state.d_point_obs_offsets.data());
  int* p_point_obs_list = thrust::raw_pointer_cast(state.d_point_obs_list.data());

  double* p_residuals = thrust::raw_pointer_cast(state.d_residuals.data());
  double* p_J_cameras = thrust::raw_pointer_cast(state.d_J_cameras.data());
  double* p_J_points = thrust::raw_pointer_cast(state.d_J_points.data());

  double* p_U = thrust::raw_pointer_cast(state.d_U.data());
  double* p_V = thrust::raw_pointer_cast(state.d_V.data());
  double* p_W = thrust::raw_pointer_cast(state.d_W.data());
  double* p_g_c = thrust::raw_pointer_cast(state.d_g_c.data());
  double* p_g_p = thrust::raw_pointer_cast(state.d_g_p.data());

  double* p_U_damped = thrust::raw_pointer_cast(state.d_U_damped.data());
  double* p_V_damped = thrust::raw_pointer_cast(state.d_V_damped.data());

  double* p_S = thrust::raw_pointer_cast(state.d_S.data());
  double* p_rhs = thrust::raw_pointer_cast(state.d_rhs.data());
  double* p_S_factored = thrust::raw_pointer_cast(state.d_S_factored.data());

  double* p_delta_cameras = thrust::raw_pointer_cast(state.d_delta_cameras.data());
  double* p_delta_points = thrust::raw_pointer_cast(state.d_delta_points.data());

  double* p_saved_cameras = thrust::raw_pointer_cast(state.d_saved_cameras.data());
  double* p_saved_points = thrust::raw_pointer_cast(state.d_saved_points.data());

  double* p_sq_residuals = thrust::raw_pointer_cast(state.d_sq_residuals.data());

  double* p_cusolver_workspace = thrust::raw_pointer_cast(state.d_cusolver_workspace.data());
  int* p_cusolver_info = thrust::raw_pointer_cast(state.d_cusolver_info.data());

  // ---- Convert CPU LossType to GPU LossType ----
  // The CPU enum (solver::LossType) and GPU enum (gpu::LossType) have the same
  // integer values but live in different namespaces. Cast via int.
  gpu::LossType gpu_loss_type =
      static_cast<gpu::LossType>(static_cast<int>(config.loss_type));
  double loss_param = config.loss_param;

  // ---- Compute initial cost on GPU ----
  gpu::computeSquaredResidualsKernel<<<gridSize(num_obs), BLOCK_SIZE>>>(
      p_cameras, p_points, p_observations,
      p_cam_indices, p_pt_indices, num_obs, p_sq_residuals,
      gpu_loss_type, loss_param);
  cudaDeviceSynchronize();

  double current_cost = 0.5 * thrust::reduce(
      state.d_sq_residuals.begin(), state.d_sq_residuals.end(),
      0.0, thrust::plus<double>());

  result.initial_cost = current_cost;
  double lambda = config.initial_lambda;

  if (config.verbose) {
    std::cout << "LM-GPU: initial cost = " << current_cost
              << " (RMS = " << std::sqrt(2.0 * current_cost / num_obs) << ")"
              << std::endl;
  }

  for (int iter = 0; iter < config.max_iterations; ++iter) {
    result.iterations = iter + 1;

    // ---- Early termination: cost is already zero ----
    if (current_cost <= 0.0) {
      result.converged = true;
      result.termination_reason = "Cost is zero";
      break;
    }

    // ---- Step 1: Compute residuals + Jacobians (GPU) ----
    // When use_lie is true, use the SE(3) left-perturbation Jacobian.
    // The Lie Jacobian uses -[P_cam]× for the rotation columns instead of
    // the full Rodrigues derivative, which is both simpler and correct for
    // the exponential map update T_new = Exp(δξ) · T_old.
    if (config.use_lie) {
      ops::projectionResidualAndJacobianLie<<<gridSize(num_obs), BLOCK_SIZE>>>(
          p_cameras, p_points, p_observations,
          p_cam_indices, p_pt_indices, num_obs,
          p_residuals, p_J_cameras, p_J_points);
    } else {
      ops::projectionResidualAndJacobian<<<gridSize(num_obs), BLOCK_SIZE>>>(
          p_cameras, p_points, p_observations,
          p_cam_indices, p_pt_indices, num_obs,
          p_residuals, p_J_cameras, p_J_points);
    }

    // ---- Step 2: Zero normal equation accumulators ----
    cudaMemset(p_U, 0, num_cameras * gpu::U_BLOCK * sizeof(double));
    cudaMemset(p_V, 0, num_points * gpu::V_BLOCK * sizeof(double));
    cudaMemset(p_g_c, 0, num_cameras * gpu::CAM_DIM * sizeof(double));
    cudaMemset(p_g_p, 0, num_points * gpu::PT_DIM * sizeof(double));
    // W doesn't need zeroing — it's written per-observation, not accumulated

    // ---- Step 3: Accumulate normal equations (GPU) ----
    // Pass loss type and parameter for IRLS weighting inside the kernel.
    gpu::accumulateNormalEquationsKernel<<<gridSize(num_obs), BLOCK_SIZE>>>(
        p_residuals, p_J_cameras, p_J_points,
        p_cam_indices, p_pt_indices, num_obs,
        p_U, p_V, p_W, p_g_c, p_g_p,
        gpu_loss_type, loss_param);

    // ---- Step 3b: Check gradient convergence ----
    // Download g_c and g_p to check infinity norm
    // (This is a small transfer: num_cameras*9 + num_points*3 doubles)
    {
      std::vector<double> h_g_c(num_cameras * gpu::CAM_DIM);
      std::vector<double> h_g_p(num_points * gpu::PT_DIM);
      cudaMemcpy(h_g_c.data(), p_g_c,
                 h_g_c.size() * sizeof(double), cudaMemcpyDeviceToHost);
      cudaMemcpy(h_g_p.data(), p_g_p,
                 h_g_p.size() * sizeof(double), cudaMemcpyDeviceToHost);

      double max_gradient = 0.0;
      for (double v : h_g_c) max_gradient = std::max(max_gradient, std::abs(v));
      for (double v : h_g_p) max_gradient = std::max(max_gradient, std::abs(v));

      if (max_gradient < config.gradient_tolerance) {
        result.converged = true;
        result.termination_reason = "Gradient below tolerance";
        break;
      }
    }

    // ---- Inner loop: try different lambda values ----
    // The Schur complement is formed inside the retry loop because LM
    // damping must be applied to U and V BEFORE Schur formation. When
    // lambda changes on retry, U_damped and V_damped change, so the
    // entire Schur complement must be recomputed.
    bool step_accepted = false;

    // Save current parameters (D2D copy)
    cudaMemcpy(p_saved_cameras, p_cameras,
               num_cameras * gpu::CAM_DIM * sizeof(double),
               cudaMemcpyDeviceToDevice);
    cudaMemcpy(p_saved_points, p_points,
               num_points * gpu::PT_DIM * sizeof(double),
               cudaMemcpyDeviceToDevice);

    // Compute parameter norm for step size check (download saved params)
    // We already have them from the D2D copy — download from saved
    std::vector<double> h_saved_cameras(num_cameras * gpu::CAM_DIM);
    std::vector<double> h_saved_points(num_points * gpu::PT_DIM);
    cudaMemcpy(h_saved_cameras.data(), p_saved_cameras,
               h_saved_cameras.size() * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_saved_points.data(), p_saved_points,
               h_saved_points.size() * sizeof(double), cudaMemcpyDeviceToHost);

    double param_norm_sq = 0.0;
    for (double v : h_saved_cameras) param_norm_sq += v * v;
    for (double v : h_saved_points) param_norm_sq += v * v;
    double param_norm = std::sqrt(param_norm_sq);

    for (int retry = 0; retry < 10; ++retry) {
      // ==================================================================
      // Step 4: Apply LM damping to U and V blocks
      // ==================================================================
      //
      // U_damped = U + λ·diag(U)
      // V_damped = V + λ·diag(V)
      //
      // This ensures V_damped is always well-conditioned for inversion
      // in the Schur complement kernel, and the resulting Schur complement
      // S = U_damped - W · V_damped^{-1} · W^T is guaranteed SPD.
      //
      gpu::applyDampingToUKernel<<<gridSize(num_cameras), BLOCK_SIZE>>>(
          p_U, p_U_damped, num_cameras, lambda, 1e-6);
      gpu::applyDampingToVKernel<<<gridSize(num_points), BLOCK_SIZE>>>(
          p_V, p_V_damped, num_points, lambda, 1e-6);

      // ==================================================================
      // Step 5: Form Schur complement from damped U and V
      // ==================================================================
      //
      // S = U_damped - W · V_damped^{-1} · W^T
      // rhs = g_c - W · V_damped^{-1} · g_p
      //
      // No additional damping on S is needed — it's already properly
      // damped through U_damped and V_damped.
      //
      cudaMemset(p_S, 0, total_cam_dim * total_cam_dim * sizeof(double));
      cudaMemset(p_rhs, 0, total_cam_dim * sizeof(double));

      gpu::initSchurWithUKernel<<<gridSize(num_cameras), BLOCK_SIZE>>>(
          p_U_damped, p_g_c, num_cameras, total_cam_dim, p_S, p_rhs);

      gpu::schurComplementKernel<<<gridSize(num_points), BLOCK_SIZE>>>(
          p_V_damped, p_W, p_g_p,
          p_cam_indices, p_point_obs_offsets, p_point_obs_list,
          num_points, total_cam_dim, p_S, p_rhs);

      // ==================================================================
      // Step 6: Solve S · δc = -rhs on GPU via cuSOLVER Cholesky
      // ==================================================================
      //
      // cuSOLVER's dpotrf (Cholesky) factorizes in-place, so we copy
      // d_S → d_S_factored then factorize.
      //
      // IMPORTANT: Our Schur complement kernels store d_S in row-major
      // order (C convention), but cuSOLVER expects column-major (Fortran
      // convention). For a symmetric matrix A, row-major A is the same as
      // column-major A^T = A. So we use CUBLAS_FILL_MODE_UPPER: cuSOLVER
      // reads the upper triangle of what it thinks is column-major, which
      // corresponds to the upper triangle of our row-major matrix — and
      // since S is symmetric, this contains all the information needed.
      //
      // The RHS is set up as d_delta_cameras = -rhs, then dpotrs solves
      // in-place: d_delta_cameras = S^{-1} * (-rhs).
      //
      // If Cholesky fails (info > 0, matrix not SPD), we increase lambda
      // and retry. More damping on U and V makes V_damped^{-1} smaller
      // and U_damped more dominant, driving S toward SPD.
      //

      // Copy S → S_factored (working copy for in-place factorization)
      cudaMemcpy(p_S_factored, p_S,
                 total_cam_dim * total_cam_dim * sizeof(double),
                 cudaMemcpyDeviceToDevice);

      // Set up RHS: d_delta_cameras = -rhs
      negateVectorKernel<<<gridSize(total_cam_dim), BLOCK_SIZE>>>(
          p_rhs, p_delta_cameras, total_cam_dim);

      cudaDeviceSynchronize();

      // Cholesky factorization (in-place on d_S_factored).
      // CUBLAS_FILL_MODE_UPPER because our row-major S, when interpreted
      // as column-major by cuSOLVER, has the correct data in the upper
      // triangle (symmetric matrix: row-major upper = col-major upper).
      cusolverDnDpotrf(
          state.cusolver_handle, CUBLAS_FILL_MODE_UPPER,
          total_cam_dim,
          p_S_factored, total_cam_dim,
          p_cusolver_workspace, state.cusolver_workspace_size,
          p_cusolver_info);

      // Check factorization status
      int h_info = 0;
      cudaMemcpy(&h_info, p_cusolver_info, sizeof(int), cudaMemcpyDeviceToHost);

      if (h_info != 0) {
        // Cholesky failed — matrix not SPD at the h_info-th minor.
        // Increase lambda to add more diagonal dominance and retry.
        if (config.verbose) {
          std::cerr << "LM-GPU: cuSOLVER Cholesky failed (info=" << h_info
                    << ", lambda=" << lambda << "), increasing lambda"
                    << std::endl;
        }
        lambda = std::min(lambda * config.lambda_factor, config.max_lambda);
        if (lambda >= config.max_lambda) {
          result.termination_reason = "Lambda exceeded maximum";
          break;
        }
        continue;  // retry with larger lambda
      }

      // Cholesky succeeded — solve R^T * R * x = -rhs in-place.
      // d_delta_cameras already contains -rhs; dpotrs overwrites it with x.
      cusolverDnDpotrs(
          state.cusolver_handle, CUBLAS_FILL_MODE_UPPER,
          total_cam_dim,
          1,  // nrhs = 1 (single right-hand side)
          p_S_factored, total_cam_dim,
          p_delta_cameras, total_cam_dim,
          p_cusolver_info);
      cudaDeviceSynchronize();

      // ---- Step 7: Back-substitute for delta_points (GPU) ----
      // Uses V_damped (not undamped V) to be consistent with the Schur
      // complement: δp_j = -V_damped_j^{-1} · (g_p_j + Σ W_k^T · δc)
      gpu::backSubstituteKernel<<<gridSize(num_points), BLOCK_SIZE>>>(
          p_V_damped, p_W, p_g_p, p_delta_cameras,
          p_cam_indices, p_point_obs_offsets, p_point_obs_list,
          num_points, p_delta_points);

      // ---- Check step size ----
      // Download delta_cameras and delta_points to compute step norm
      std::vector<double> h_delta_cam(total_cam_dim);
      std::vector<double> h_delta_points(total_pt_dim);
      cudaMemcpy(h_delta_cam.data(), p_delta_cameras,
                 total_cam_dim * sizeof(double), cudaMemcpyDeviceToHost);
      cudaMemcpy(h_delta_points.data(), p_delta_points,
                 total_pt_dim * sizeof(double), cudaMemcpyDeviceToHost);

      double step_norm = 0.0;
      for (double v : h_delta_cam) step_norm += v * v;
      step_norm = std::sqrt(step_norm);
      for (double v : h_delta_points) step_norm += std::abs(v);
      // Use L1 for points to avoid another sqrt, just a rough check

      if (step_norm / (param_norm + 1e-12) < config.step_tolerance) {
        result.converged = true;
        result.termination_reason = "Step size below tolerance";
        break;
      }

      // ---- Step 9: Restore params from saved, then apply update ----
      // First restore (in case this is a retry after rejection)
      cudaMemcpy(p_cameras, p_saved_cameras,
                 num_cameras * gpu::CAM_DIM * sizeof(double),
                 cudaMemcpyDeviceToDevice);
      cudaMemcpy(p_points, p_saved_points,
                 num_points * gpu::PT_DIM * sizeof(double),
                 cudaMemcpyDeviceToDevice);

      // Apply update
      // When use_lie is true, camera poses are updated via the exponential
      // map on SE(3): T_new = Exp(δξ) · T_old. Intrinsics (f, k1, k2) are
      // still updated additively inside the Lie kernel.
      // Points always use additive updates regardless of use_lie.
      if (config.use_lie) {
        gpu::updateCameraParametersLieKernel<<<gridSize(num_cameras), BLOCK_SIZE>>>(
            p_cameras, p_delta_cameras, num_cameras);
      } else {
        gpu::updateParametersKernel<<<gridSize(num_cameras * gpu::CAM_DIM), BLOCK_SIZE>>>(
            p_cameras, p_delta_cameras, num_cameras * gpu::CAM_DIM);
      }
      gpu::updateParametersKernel<<<gridSize(num_points * gpu::PT_DIM), BLOCK_SIZE>>>(
          p_points, p_delta_points, num_points * gpu::PT_DIM);

      // ---- Step 10: Compute new cost (GPU) ----
      gpu::computeSquaredResidualsKernel<<<gridSize(num_obs), BLOCK_SIZE>>>(
          p_cameras, p_points, p_observations,
          p_cam_indices, p_pt_indices, num_obs, p_sq_residuals,
          gpu_loss_type, loss_param);
      cudaDeviceSynchronize();

      double new_cost = 0.5 * thrust::reduce(
          state.d_sq_residuals.begin(), state.d_sq_residuals.end(),
          0.0, thrust::plus<double>());

      // ---- Step 11: Accept or reject ----
      if (new_cost < current_cost) {
        double cost_change = current_cost - new_cost;

        if (config.verbose) {
          std::cout << "LM-GPU iter " << iter + 1 << ": cost " << current_cost
                    << " -> " << new_cost << " (delta=" << cost_change
                    << ", RMS=" << std::sqrt(2.0 * new_cost / num_obs)
                    << ", lambda=" << lambda << ")" << std::endl;
        }

        current_cost = new_cost;
        lambda = std::max(lambda / config.lambda_factor, config.min_lambda);
        step_accepted = true;

        if (cost_change / (current_cost + 1e-12) < config.cost_tolerance) {
          result.converged = true;
          result.termination_reason = "Cost change below tolerance";
        }
        break;
      } else {
        // Reject: restore parameters from saved
        cudaMemcpy(p_cameras, p_saved_cameras,
                   num_cameras * gpu::CAM_DIM * sizeof(double),
                   cudaMemcpyDeviceToDevice);
        cudaMemcpy(p_points, p_saved_points,
                   num_points * gpu::PT_DIM * sizeof(double),
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

  // ---- Download final parameters to host ----
  state.downloadToHost(problem);

  if (config.verbose) {
    std::cout << "LM-GPU: " << result.termination_reason << " after "
              << result.iterations << " iterations"
              << ". Final cost = " << result.final_cost
              << " (RMS = " << std::sqrt(2.0 * result.final_cost / num_obs)
              << ")" << std::endl;
  }

  return result;
}

}  // namespace solver
}  // namespace backend
}  // namespace substral
