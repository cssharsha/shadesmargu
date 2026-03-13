#pragma once

#include <Eigen/Dense>
#include <vector>

#include "subastral/backend/common.h"

namespace substral {
namespace backend {
namespace solver {

// Camera block size (angle-axis[3] + translation[3] + f + k1 + k2)
static constexpr int CAM_DIM = 9;
// Point block size (X, Y, Z)
static constexpr int PT_DIM = 3;
// Residual dimension per observation
static constexpr int RES_DIM = 2;

// =============================================================================
// Block-Sparse Normal Equations for Bundle Adjustment
// =============================================================================
//
// The full Hessian of the BA problem has the structure:
//
//   H = J^T J = | U    W  |     g = J^T r = | g_c |
//               | W^T  V  |                  | g_p |
//
// where:
//   U[i] = sum over observations seeing camera i: J_ci^T J_ci    (9×9)
//   V[j] = sum over observations seeing point j:  J_pj^T J_pj    (3×3)
//   W[i][j] = J_ci^T J_pj   for observation (i,j)                (9×3)
//
//   g_c[i] = sum over observations seeing camera i: J_ci^T r      (9×1)
//   g_p[j] = sum over observations seeing point j:  J_pj^T r      (3×1)
//
// U and V are block-diagonal. W is sparse — only blocks (i,j) where
// camera i observes point j are nonzero.
//
// =============================================================================

// Storage for the block-sparse normal equations.
struct NormalEquations {
  int num_cameras;
  int num_points;

  // U[i]: 9×9 symmetric block for camera i
  std::vector<Eigen::Matrix<double, CAM_DIM, CAM_DIM>> U;

  // V[j]: 3×3 symmetric block for point j
  std::vector<Eigen::Matrix<double, PT_DIM, PT_DIM>> V;

  // W: sparse camera-point cross terms.
  // Indexed by observation: W_obs[obs_idx] = J_cam^T * J_pt (9×3)
  // We also store the camera/point indices for each observation.
  std::vector<Eigen::Matrix<double, CAM_DIM, PT_DIM>> W_obs;
  std::vector<int> obs_camera_indices;
  std::vector<int> obs_point_indices;

  // Gradient vectors
  std::vector<Eigen::Matrix<double, CAM_DIM, 1>> g_cameras;  // g_c[i]
  std::vector<Eigen::Matrix<double, PT_DIM, 1>> g_points;    // g_p[j]

  void resize(int n_cameras, int n_points, int n_observations) {
    num_cameras = n_cameras;
    num_points = n_points;

    U.assign(n_cameras, Eigen::Matrix<double, CAM_DIM, CAM_DIM>::Zero());
    V.assign(n_points, Eigen::Matrix<double, PT_DIM, PT_DIM>::Zero());
    W_obs.resize(n_observations);
    obs_camera_indices.resize(n_observations);
    obs_point_indices.resize(n_observations);

    g_cameras.assign(n_cameras, Eigen::Matrix<double, CAM_DIM, 1>::Zero());
    g_points.assign(n_points, Eigen::Matrix<double, PT_DIM, 1>::Zero());
  }

  void setZero() {
    for (auto& u : U) u.setZero();
    for (auto& v : V) v.setZero();
    for (auto& w : W_obs) w.setZero();
    for (auto& gc : g_cameras) gc.setZero();
    for (auto& gp : g_points) gp.setZero();
  }
};

// =============================================================================
// Accumulate normal equations from per-observation Jacobians and residuals
// =============================================================================
//
// Input:
//   residuals:  num_obs × 2 (row-major, contiguous)
//   J_cameras:  num_obs × 18 (2×9 Jacobian per obs, row-major)
//   J_points:   num_obs × 6  (2×3 Jacobian per obs, row-major)
//   problem:    BAProblem with camera/point/observation metadata
//
// Output:
//   neq:        populated NormalEquations struct
//
// For each observation k that relates camera i to point j:
//   U[i]  += J_ck^T · J_ck        (9×9)
//   V[j]  += J_pk^T · J_pk        (3×3)
//   W_obs[k] = J_ck^T · J_pk      (9×3)
//   g_c[i] += J_ck^T · r_k        (9×1)
//   g_p[j] += J_pk^T · r_k        (3×1)
//
void accumulateNormalEquations(const BAProblem& problem,
                               const double* residuals,  // num_obs * 2
                               const double* J_cameras,  // num_obs * 18
                               const double* J_points,   // num_obs * 6
                               NormalEquations& neq);

// =============================================================================
// Schur Complement: Reduce to camera-only system
// =============================================================================
//
// Given the normal equations:
//   | U    W  | | δc |   | g_c |
//   | W^T  V  | | δp | = | g_p |
//
// The Schur complement eliminates the point variables:
//
//   S = U - W · V^{-1} · W^T           (dense num_cameras*9 × num_cameras*9)
//   b = g_c - W · V^{-1} · g_p         (num_cameras*9 × 1)
//
// Then solve: (S + λ·I) · δc = -b     (with LM damping λ)
//
// Since V is block-diagonal with 3×3 blocks, V^{-1} is trivial.
//
// Note: S is dense because in typical BA problems, many cameras share
// points, creating fill-in. For small-to-medium problems (< 1000 cameras),
// a dense solve is practical. For larger problems, PCG would be used.
//
// Output:
//   S:   dense (num_cameras*9) × (num_cameras*9) matrix
//   rhs: dense (num_cameras*9) × 1 vector
//
void computeSchurComplement(
    const NormalEquations& neq,
    Eigen::MatrixXd& S,     // output: (num_cam*9) × (num_cam*9)
    Eigen::VectorXd& rhs);  // output: (num_cam*9) × 1

// =============================================================================
// Back-substitution: recover point updates from camera updates
// =============================================================================
//
// Given δc (camera updates) from solving the reduced system:
//   δp_j = V_j^{-1} · (g_p_j - sum_{obs k seeing j} W_obs[k]^T · δc_{cam(k)})
//
// But we want: δp_j = -V_j^{-1} · (g_p_j + W^T · δc)_j
// because the system is (S + λI)·δc = -rhs, so δc already has the right sign.
//
// More precisely:
//   δp_j = -V_j^{-1} · (g_p_j + sum_{obs k seeing j} W_obs[k]^T · δc_{cam(k)})
//
void backSubstitute(const NormalEquations& neq,
                    const Eigen::VectorXd& delta_cameras,  // (num_cam*9) × 1
                    Eigen::VectorXd& delta_points);  // output: (num_pt*3) × 1

}  // namespace solver
}  // namespace backend
}  // namespace substral
