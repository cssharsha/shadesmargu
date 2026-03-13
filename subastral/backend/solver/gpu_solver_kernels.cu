#include "subastral/backend/solver/gpu_solver_kernels.cuh"
#include "subastral/backend/lie/so3_gpu.cuh"
#include "subastral/backend/lie/se3_gpu.cuh"
#include "subastral/backend/ops/projection_gpu.cuh"

namespace substral {
namespace backend {
namespace solver {
namespace gpu {

// =============================================================================
// Kernel 1: Normal Equation Accumulation
// =============================================================================
//
// Each thread handles one observation k.
//
// J_ck is 2x9 row-major:  J_cameras[k*18 .. k*18+17]
//   Row 0: J_cameras[k*18 + 0..8]   = J_ck(0, 0..8)
//   Row 1: J_cameras[k*18 + 9..17]  = J_ck(1, 0..8)
//
// J_pk is 2x3 row-major:  J_points[k*6 .. k*6+5]
//   Row 0: J_points[k*6 + 0..2]     = J_pk(0, 0..2)
//   Row 1: J_points[k*6 + 3..5]     = J_pk(1, 0..2)
//
// U[i] += J_ck^T * J_ck:
//   U[i](r,c) = sum over m=0,1: J_ck(m,r) * J_ck(m,c)
//   J_ck(m,r) = J_cameras[k*18 + m*9 + r]
//
// V[j] += J_pk^T * J_pk:
//   V[j](r,c) = sum over m=0,1: J_pk(m,r) * J_pk(m,c)
//
// W[k] = J_ck^T * J_pk:
//   W[k](r,c) = sum over m=0,1: J_ck(m,r) * J_pk(m,c)
//
// g_c[i] += J_ck^T * r_k:
//   g_c[i](r) = sum over m=0,1: J_ck(m,r) * r_k(m)
//
// g_p[j] += J_pk^T * r_k:
//   g_p[j](r) = sum over m=0,1: J_pk(m,r) * r_k(m)
//
// =============================================================================

__global__ void accumulateNormalEquationsKernel(
    const double* __restrict__ residuals,
    const double* __restrict__ J_cameras,
    const double* __restrict__ J_points,
    const int* __restrict__ camera_indices,
    const int* __restrict__ point_indices,
    int num_obs,
    double* d_U,
    double* d_V,
    double* d_W,
    double* d_g_c,
    double* d_g_p,
    LossType loss_type,
    double loss_param) {
  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= num_obs) return;

  int cam_idx = camera_indices[k];
  int pt_idx = point_indices[k];

  const double* J_ck = &J_cameras[k * J_CAM_BLOCK];  // 2x9 row-major
  const double* J_pk = &J_points[k * J_PT_BLOCK];     // 2x3 row-major
  double r0 = residuals[k * 2 + 0];
  double r1 = residuals[k * 2 + 1];

  // ---- IRLS weighting for robust loss ----
  // Compute squared residual norm s = ||r_k||^2, then weight w = rho'(s).
  // Scale residuals and Jacobians by sqrt(w) so the standard normal equations
  // J'^T J' delta = -J'^T r' give the correct robust update.
  // For TrivialLoss, w = 1 and sqrt_w = 1, so this is a no-op.
  double s = r0 * r0 + r1 * r1;
  double w = evalWeight(loss_type, s, loss_param);
  double sqrt_w = sqrt(w);

  r0 *= sqrt_w;
  r1 *= sqrt_w;

  // We scale J_ck and J_pk entries inline below rather than modifying the
  // input arrays (which are read-only). The products J'^T J' and J'^T r'
  // pick up a factor of w from the two sqrt_w factors.

  // ---- U[cam_idx] += (sqrt_w * J_ck)^T * (sqrt_w * J_ck) = w * J_ck^T * J_ck ----
  double* U_i = &d_U[cam_idx * U_BLOCK];
  for (int r = 0; r < CAM_DIM; ++r) {
    for (int c = 0; c < CAM_DIM; ++c) {
      double val = w * (J_ck[0 * CAM_DIM + r] * J_ck[0 * CAM_DIM + c] +
                        J_ck[1 * CAM_DIM + r] * J_ck[1 * CAM_DIM + c]);
      atomicAdd(&U_i[r * CAM_DIM + c], val);
    }
  }

  // ---- V[pt_idx] += w * J_pk^T * J_pk (3x3) ----
  double* V_j = &d_V[pt_idx * V_BLOCK];
  for (int r = 0; r < PT_DIM; ++r) {
    for (int c = 0; c < PT_DIM; ++c) {
      double val = w * (J_pk[0 * PT_DIM + r] * J_pk[0 * PT_DIM + c] +
                        J_pk[1 * PT_DIM + r] * J_pk[1 * PT_DIM + c]);
      atomicAdd(&V_j[r * PT_DIM + c], val);
    }
  }

  // ---- W[k] = w * J_ck^T * J_pk (9x3) ---- (direct write, no atomics)
  double* W_k = &d_W[k * W_BLOCK];
  for (int r = 0; r < CAM_DIM; ++r) {
    for (int c = 0; c < PT_DIM; ++c) {
      W_k[r * PT_DIM + c] = w * (J_ck[0 * CAM_DIM + r] * J_pk[0 * PT_DIM + c] +
                                  J_ck[1 * CAM_DIM + r] * J_pk[1 * PT_DIM + c]);
    }
  }

  // ---- g_c[cam_idx] += (sqrt_w * J_ck)^T * (sqrt_w * r_k) = w * J_ck^T * r_k ----
  // Note: r0, r1 are already scaled by sqrt_w, so J_ck^T * [r0,r1] * sqrt_w = correct
  // Actually, let's be explicit: g_c += sqrt_w * J_ck^T * [r0, r1]
  // where r0, r1 already have sqrt_w baked in. So we need another sqrt_w on J.
  // Equivalently: g_c += w * J_ck_orig^T * r_orig. Let's use the w formulation.
  double r0_orig = residuals[k * 2 + 0];
  double r1_orig = residuals[k * 2 + 1];
  double* gc_i = &d_g_c[cam_idx * CAM_DIM];
  for (int r = 0; r < CAM_DIM; ++r) {
    double val = w * (J_ck[0 * CAM_DIM + r] * r0_orig + J_ck[1 * CAM_DIM + r] * r1_orig);
    atomicAdd(&gc_i[r], val);
  }

  // ---- g_p[pt_idx] += w * J_pk^T * r_k (3x1) ----
  double* gp_j = &d_g_p[pt_idx * PT_DIM];
  for (int r = 0; r < PT_DIM; ++r) {
    double val = w * (J_pk[0 * PT_DIM + r] * r0_orig + J_pk[1 * PT_DIM + r] * r1_orig);
    atomicAdd(&gp_j[r], val);
  }
}

// =============================================================================
// Kernel 2: Initialize Schur system with U diagonal blocks and g_c
// =============================================================================

__global__ void initSchurWithUKernel(
    const double* __restrict__ d_U,
    const double* __restrict__ d_g_c,
    int num_cameras,
    int total_cam_dim,
    double* d_S,
    double* d_rhs) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_cameras) return;

  // Copy U[i] (9x9) into S diagonal block at (i*9, i*9)
  const double* U_i = &d_U[i * U_BLOCK];
  for (int r = 0; r < CAM_DIM; ++r) {
    for (int c = 0; c < CAM_DIM; ++c) {
      d_S[(i * CAM_DIM + r) * total_cam_dim + (i * CAM_DIM + c)] =
          U_i[r * CAM_DIM + c];
    }
  }

  // Copy g_c[i] (9x1) into rhs at offset i*9
  const double* gc_i = &d_g_c[i * CAM_DIM];
  for (int d = 0; d < CAM_DIM; ++d) {
    d_rhs[i * CAM_DIM + d] = gc_i[d];
  }
}

// =============================================================================
// Kernel 3: Schur Complement Formation
// =============================================================================
//
// One thread per point j. For each point:
//   1. Invert V_j (3x3) via Cholesky: L L^T = V_j, then V_j^{-1}
//   2. For each observation k seeing j:
//      E_k = W[k] * V_j^{-1}   (9x3)
//   3. For each pair (k,l) of observations seeing j:
//      S(cam(k), cam(l)) -= E_k * W[l]^T   (9x9 block)
//   4. For each observation k seeing j:
//      rhs(cam(k)) -= E_k * g_p[j]   (9x1)
//
// 3x3 Cholesky inversion is done inline (no Eigen on device).
//
// =============================================================================

// Helper: 3x3 Cholesky decomposition L such that L*L^T = A
// A and L are row-major 3x3. Returns false if not positive definite.
__device__ inline bool cholesky3x3(const double* A, double* L) {
  // L[0][0]
  double v = A[0];
  if (v <= 0.0) return false;
  L[0] = sqrt(v);
  double inv_L00 = 1.0 / L[0];

  // L[1][0], L[1][1]
  L[3] = A[3] * inv_L00;  // L[1][0] = A[1][0] / L[0][0]
  v = A[4] - L[3] * L[3]; // A[1][1] - L[1][0]^2
  if (v <= 0.0) return false;
  L[4] = sqrt(v);
  double inv_L11 = 1.0 / L[4];

  // L[2][0], L[2][1], L[2][2]
  L[6] = A[6] * inv_L00;                       // L[2][0] = A[2][0] / L[0][0]
  L[7] = (A[7] - L[6] * L[3]) * inv_L11;       // L[2][1]
  v = A[8] - L[6] * L[6] - L[7] * L[7];        // A[2][2] - L[2][0]^2 - L[2][1]^2
  if (v <= 0.0) return false;
  L[8] = sqrt(v);

  // Zero upper triangle
  L[1] = 0.0; L[2] = 0.0; L[5] = 0.0;

  return true;
}

// Helper: Solve L*x = b for x (forward substitution), L is 3x3 lower triangular
__device__ inline void forwardSolve3(const double* L, const double* b, double* x) {
  x[0] = b[0] / L[0];
  x[1] = (b[1] - L[3] * x[0]) / L[4];
  x[2] = (b[2] - L[6] * x[0] - L[7] * x[1]) / L[8];
}

// Helper: Solve L^T*x = b for x (backward substitution), L is 3x3 lower triangular
__device__ inline void backwardSolve3(const double* L, const double* b, double* x) {
  x[2] = b[2] / L[8];
  x[1] = (b[1] - L[7] * x[2]) / L[4];
  x[0] = (b[0] - L[3] * x[1] - L[6] * x[2]) / L[0];
}

// Helper: Compute V^{-1} given Cholesky factor L: V^{-1} = L^{-T} L^{-1}
// Solves V * X = I column by column
__device__ inline void invertViaCholesky3(const double* L, double* Vinv) {
  for (int col = 0; col < 3; ++col) {
    double e[3] = {0.0, 0.0, 0.0};
    e[col] = 1.0;
    double y[3], x[3];
    forwardSolve3(L, e, y);
    backwardSolve3(L, y, x);
    Vinv[0 * 3 + col] = x[0];
    Vinv[1 * 3 + col] = x[1];
    Vinv[2 * 3 + col] = x[2];
  }
}

__global__ void schurComplementKernel(
    const double* __restrict__ d_V,
    const double* __restrict__ d_W,
    const double* __restrict__ d_g_p,
    const int* __restrict__ camera_indices,
    const int* __restrict__ point_obs_offsets,
    const int* __restrict__ point_obs_list,
    int num_points,
    int total_cam_dim,
    double* d_S,
    double* d_rhs) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (j >= num_points) return;

  int obs_start = point_obs_offsets[j];
  int obs_end = point_obs_offsets[j + 1];
  int n_obs = obs_end - obs_start;
  if (n_obs == 0) return;

  // ---- Step 1: Invert V_j via Cholesky ----
  const double* V_j = &d_V[j * V_BLOCK];
  double L[9];
  double Vinv[9];  // V_j^{-1}, 3x3 row-major

  if (!cholesky3x3(V_j, L)) {
    // V_j is not positive definite — skip this point.
    // This shouldn't happen for well-posed problems.
    return;
  }
  invertViaCholesky3(L, Vinv);

  // ---- Step 2: Load g_p[j] ----
  double gp[3];
  gp[0] = d_g_p[j * PT_DIM + 0];
  gp[1] = d_g_p[j * PT_DIM + 1];
  gp[2] = d_g_p[j * PT_DIM + 2];

  // Precompute Vinv * gp (3x1)
  double Vinv_gp[3];
  for (int r = 0; r < 3; ++r) {
    Vinv_gp[r] = Vinv[r * 3 + 0] * gp[0] +
                 Vinv[r * 3 + 1] * gp[1] +
                 Vinv[r * 3 + 2] * gp[2];
  }

  // ---- Step 3: For each observation k seeing j ----
  for (int idx_k = 0; idx_k < n_obs; ++idx_k) {
    int k = point_obs_list[obs_start + idx_k];
    int cam_k = camera_indices[k];
    const double* W_k = &d_W[k * W_BLOCK];  // 9x3 row-major

    // E_k = W_k * Vinv  (9x3 * 3x3 = 9x3)
    double E_k[W_BLOCK];  // 9x3
    for (int r = 0; r < CAM_DIM; ++r) {
      for (int c = 0; c < PT_DIM; ++c) {
        E_k[r * PT_DIM + c] = W_k[r * PT_DIM + 0] * Vinv[0 * 3 + c] +
                               W_k[r * PT_DIM + 1] * Vinv[1 * 3 + c] +
                               W_k[r * PT_DIM + 2] * Vinv[2 * 3 + c];
      }
    }

    // rhs(cam_k) -= E_k * g_p[j]  = W_k * Vinv * gp  (9x3 * 3x1 = 9x1)
    // Equivalently: E_k * gp, but we already have Vinv_gp = Vinv * gp
    // So: rhs -= W_k * Vinv_gp
    for (int r = 0; r < CAM_DIM; ++r) {
      double val = W_k[r * PT_DIM + 0] * Vinv_gp[0] +
                   W_k[r * PT_DIM + 1] * Vinv_gp[1] +
                   W_k[r * PT_DIM + 2] * Vinv_gp[2];
      atomicAdd(&d_rhs[cam_k * CAM_DIM + r], -val);
    }

    // S(cam_k, cam_l) -= E_k * W_l^T for each l seeing j
    for (int idx_l = 0; idx_l < n_obs; ++idx_l) {
      int l = point_obs_list[obs_start + idx_l];
      int cam_l = camera_indices[l];
      const double* W_l = &d_W[l * W_BLOCK];  // 9x3 row-major

      // E_k * W_l^T: (9x3) * (3x9) = (9x9)
      // E_k(r, m) * W_l(c, m) summed over m=0..2
      for (int r = 0; r < CAM_DIM; ++r) {
        for (int c = 0; c < CAM_DIM; ++c) {
          double val = E_k[r * PT_DIM + 0] * W_l[c * PT_DIM + 0] +
                       E_k[r * PT_DIM + 1] * W_l[c * PT_DIM + 1] +
                       E_k[r * PT_DIM + 2] * W_l[c * PT_DIM + 2];
          atomicAdd(
              &d_S[(cam_k * CAM_DIM + r) * total_cam_dim + (cam_l * CAM_DIM + c)],
              -val);
        }
      }
    }
  }
}

// =============================================================================
// Kernel 4: Apply LM Damping
// =============================================================================

__global__ void applyLMDampingKernel(
    double* d_S,
    int total_cam_dim,
    double lambda,
    double min_diag) {
  int d = blockIdx.x * blockDim.x + threadIdx.x;
  if (d >= total_cam_dim) return;

  double diag_val = d_S[d * total_cam_dim + d];
  double damping = lambda * fmax(diag_val, min_diag);
  d_S[d * total_cam_dim + d] = diag_val + damping;
}

// =============================================================================
// Kernel 4b: Apply LM Damping to U blocks
// =============================================================================
//
// One thread per camera. Copies U[i] → U_damped[i] then adds damping to
// the 9 diagonal entries (row-major indices 0, 10, 20, 30, 40, 50, 60, 70, 80).
//
// =============================================================================

__global__ void applyDampingToUKernel(
    const double* __restrict__ d_U,
    double* d_U_damped,
    int num_cameras,
    double lambda,
    double min_diag) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_cameras) return;

  const double* src = &d_U[i * U_BLOCK];
  double* dst = &d_U_damped[i * U_BLOCK];

  // Copy entire 9x9 block
  for (int k = 0; k < U_BLOCK; ++k) {
    dst[k] = src[k];
  }

  // Add damping to diagonal: row-major index (d, d) = d * CAM_DIM + d
  for (int d = 0; d < CAM_DIM; ++d) {
    int diag_idx = d * CAM_DIM + d;
    double diag_val = dst[diag_idx];
    dst[diag_idx] = diag_val + lambda * fmax(diag_val, min_diag);
  }
}

// =============================================================================
// Kernel 4c: Apply LM Damping to V blocks
// =============================================================================
//
// One thread per point. Copies V[j] → V_damped[j] then adds damping to
// the 3 diagonal entries (row-major indices 0, 4, 8).
//
// =============================================================================

__global__ void applyDampingToVKernel(
    const double* __restrict__ d_V,
    double* d_V_damped,
    int num_points,
    double lambda,
    double min_diag) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (j >= num_points) return;

  const double* src = &d_V[j * V_BLOCK];
  double* dst = &d_V_damped[j * V_BLOCK];

  // Copy entire 3x3 block
  for (int k = 0; k < V_BLOCK; ++k) {
    dst[k] = src[k];
  }

  // Add damping to diagonal: row-major index (d, d) = d * PT_DIM + d
  for (int d = 0; d < PT_DIM; ++d) {
    int diag_idx = d * PT_DIM + d;
    double diag_val = dst[diag_idx];
    dst[diag_idx] = diag_val + lambda * fmax(diag_val, min_diag);
  }
}

// =============================================================================
// Kernel 5: Back-Substitution
// =============================================================================
//
// One thread per point j:
//   rhs_j = g_p_j + sum_{k seeing j} W[k]^T * delta_c_{cam(k)}
//   delta_p_j = -V_j^{-1} * rhs_j
//
// =============================================================================

__global__ void backSubstituteKernel(
    const double* __restrict__ d_V,
    const double* __restrict__ d_W,
    const double* __restrict__ d_g_p,
    const double* __restrict__ d_delta_cameras,
    const int* __restrict__ camera_indices,
    const int* __restrict__ point_obs_offsets,
    const int* __restrict__ point_obs_list,
    int num_points,
    double* d_delta_points) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (j >= num_points) return;

  int obs_start = point_obs_offsets[j];
  int obs_end = point_obs_offsets[j + 1];

  // rhs_j = g_p_j
  double rhs[3];
  rhs[0] = d_g_p[j * PT_DIM + 0];
  rhs[1] = d_g_p[j * PT_DIM + 1];
  rhs[2] = d_g_p[j * PT_DIM + 2];

  // rhs_j += sum_{k seeing j} W[k]^T * delta_c_{cam(k)}
  for (int idx = obs_start; idx < obs_end; ++idx) {
    int k = point_obs_list[idx];
    int cam_idx = camera_indices[k];
    const double* W_k = &d_W[k * W_BLOCK];  // 9x3 row-major
    const double* dc = &d_delta_cameras[cam_idx * CAM_DIM];

    // W_k^T * dc: (3x9) * (9x1) = (3x1)
    // W_k^T(r, m) = W_k(m, r), so: sum over m: W_k(m, r) * dc(m)
    for (int r = 0; r < PT_DIM; ++r) {
      double val = 0.0;
      for (int m = 0; m < CAM_DIM; ++m) {
        val += W_k[m * PT_DIM + r] * dc[m];
      }
      rhs[r] += val;
    }
  }

  // delta_p_j = -V_j^{-1} * rhs_j
  // Solve V_j * x = rhs_j via Cholesky, then delta_p = -x
  const double* V_j = &d_V[j * V_BLOCK];
  double L[9];
  if (!cholesky3x3(V_j, L)) {
    // Not positive definite — set delta to zero
    d_delta_points[j * PT_DIM + 0] = 0.0;
    d_delta_points[j * PT_DIM + 1] = 0.0;
    d_delta_points[j * PT_DIM + 2] = 0.0;
    return;
  }

  double y[3], x[3];
  forwardSolve3(L, rhs, y);
  backwardSolve3(L, y, x);

  d_delta_points[j * PT_DIM + 0] = -x[0];
  d_delta_points[j * PT_DIM + 1] = -x[1];
  d_delta_points[j * PT_DIM + 2] = -x[2];
}

// =============================================================================
// Kernel 6: Parameter Update
// =============================================================================

__global__ void updateParametersKernel(
    double* d_params,
    const double* __restrict__ d_delta,
    int total_elements) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= total_elements) return;
  d_params[i] += d_delta[i];
}

// =============================================================================
// Kernel 6b: Lie Group Camera Parameter Update
// =============================================================================
//
// One thread per camera. Applies SE(3) exponential map update to pose
// and additive update to intrinsics.
//
// =============================================================================

__global__ void updateCameraParametersLieKernel(
    double* d_cameras,
    const double* __restrict__ d_delta_cameras,
    int num_cameras) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_cameras) return;

  double* cam = &d_cameras[i * CAM_DIM];
  const double* delta = &d_delta_cameras[i * CAM_DIM];

  // ---- Pose update via SE(3) exponential map ----
  //
  // Current pose: ω_old = cam[0..2], t_old = cam[3..5]
  // Delta: δξ = (δφ, δρ) = delta[0..5]
  //
  // T_new = Exp(δξ) · T_old
  //   where T_old = (Exp_SO3(ω_old), t_old)

  // Step 1: R_old = Exp_SO3(ω_old)
  double R_old[9];
  lie::device::expSO3(cam, R_old);

  // Step 2: (R_delta, t_delta) = Exp_SE3(δξ)
  double R_delta[9], t_delta[3];
  lie::device::expSE3(delta, R_delta, t_delta);

  // Step 3: Compose: (R_new, t_new) = (R_delta, t_delta) · (R_old, t_old)
  //   R_new = R_delta · R_old
  //   t_new = R_delta · t_old + t_delta
  double R_new[9], t_new[3];
  lie::device::composeSE3(R_delta, t_delta, R_old, cam + 3, R_new, t_new);

  // Step 4: ω_new = Log_SO3(R_new)
  double w_new[3];
  lie::device::logSO3(R_new, w_new);

  // Step 5: Write back pose
  cam[0] = w_new[0];
  cam[1] = w_new[1];
  cam[2] = w_new[2];
  cam[3] = t_new[0];
  cam[4] = t_new[1];
  cam[5] = t_new[2];

  // ---- Intrinsics update (additive) ----
  cam[6] += delta[6];  // f
  cam[7] += delta[7];  // k1
  cam[8] += delta[8];  // k2
}

// =============================================================================
// Host wrapper for Lie camera update kernel
// =============================================================================

void launchUpdateCameraParametersLie(
    double* d_cameras,
    const double* d_delta_cameras,
    int num_cameras) {
  int threads = 256;
  int blocks = (num_cameras + threads - 1) / threads;
  updateCameraParametersLieKernel<<<blocks, threads>>>(
      d_cameras, d_delta_cameras, num_cameras);
  cudaDeviceSynchronize();
}

// =============================================================================
// Kernel 7: Squared Residuals for Cost
// =============================================================================
//
// Lightweight projection + squared residual without Jacobian computation.
// Reuses the device::projectPoint function from projection_gpu.cuh.
//
// =============================================================================

__global__ void computeSquaredResidualsKernel(
    const double* __restrict__ d_cameras,
    const double* __restrict__ d_points,
    const double* __restrict__ d_observations,
    const int* __restrict__ camera_indices,
    const int* __restrict__ point_indices,
    int num_obs,
    double* d_sq_residuals,
    LossType loss_type,
    double loss_param) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_obs) return;

  int cam_idx = camera_indices[i];
  int pt_idx = point_indices[i];

  const double* cam = &d_cameras[cam_idx * 9];
  const double* pt = &d_points[pt_idx * 3];
  const double* obs = &d_observations[i * 2];

  double predicted[2];
  ops::device::projectPoint(cam, pt, predicted);

  double rx = predicted[0] - obs[0];
  double ry = predicted[1] - obs[1];
  double s = rx * rx + ry * ry;

  // Apply robust loss: store rho(s) instead of raw s.
  // For TrivialLoss, rho(s) = s, so this is backward-compatible.
  // The caller computes total cost = 0.5 * sum(rho(s_i)).
  d_sq_residuals[i] = evalRho(loss_type, s, loss_param);
}

}  // namespace gpu
}  // namespace solver
}  // namespace backend
}  // namespace substral
