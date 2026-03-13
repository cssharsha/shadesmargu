#include "subastral/backend/solver/schur.hpp"

#include <Eigen/Dense>

namespace substral {
namespace backend {
namespace solver {

// =============================================================================
// Accumulate Normal Equations
// =============================================================================

void accumulateNormalEquations(const BAProblem& problem,
                               const double* residuals, const double* J_cameras,
                               const double* J_points, NormalEquations& neq) {
  int num_obs = static_cast<int>(problem.observations.size());
  int num_cameras = static_cast<int>(problem.cameras.size());
  int num_points = static_cast<int>(problem.points.size());

  neq.resize(num_cameras, num_points, num_obs);
  neq.setZero();

  for (int k = 0; k < num_obs; ++k) {
    int cam_idx = problem.observations[k]->get_camera_id();
    int pt_idx = problem.observations[k]->get_point_id();

    // Map the per-observation Jacobian blocks (row-major in memory)
    // J_ck is 2×9, J_pk is 2×3, r_k is 2×1
    Eigen::Map<const Eigen::Matrix<double, RES_DIM, CAM_DIM, Eigen::RowMajor>>
        J_ck(J_cameras + k * (RES_DIM * CAM_DIM));
    Eigen::Map<const Eigen::Matrix<double, RES_DIM, PT_DIM, Eigen::RowMajor>>
        J_pk(J_points + k * (RES_DIM * PT_DIM));
    Eigen::Map<const Eigen::Vector2d> r_k(residuals + k * RES_DIM);

    // U[cam_idx] += J_ck^T · J_ck   (9×2 · 2×9 = 9×9)
    neq.U[cam_idx].noalias() += J_ck.transpose() * J_ck;

    // V[pt_idx] += J_pk^T · J_pk    (3×2 · 2×3 = 3×3)
    neq.V[pt_idx].noalias() += J_pk.transpose() * J_pk;

    // W_obs[k] = J_ck^T · J_pk      (9×2 · 2×3 = 9×3)
    neq.W_obs[k].noalias() = J_ck.transpose() * J_pk;
    neq.obs_camera_indices[k] = cam_idx;
    neq.obs_point_indices[k] = pt_idx;

    // g_c[cam_idx] += J_ck^T · r_k  (9×2 · 2×1 = 9×1)
    neq.g_cameras[cam_idx].noalias() += J_ck.transpose() * r_k;

    // g_p[pt_idx] += J_pk^T · r_k   (3×2 · 2×1 = 3×1)
    neq.g_points[pt_idx].noalias() += J_pk.transpose() * r_k;
  }
}

// =============================================================================
// Schur Complement
// =============================================================================
//
// S = U - sum_k W_obs[k] · V[pt(k)]^{-1} · W_obs[k]^T
// rhs = g_c - sum_k W_obs[k] · V[pt(k)]^{-1} · g_p[pt(k)]
//
// We iterate over observations. For each observation k relating camera i
// to point j:
//   - Compute V_j^{-1} (3×3 inverse, cached per point)
//   - Compute E_k = W_obs[k] · V_j^{-1}  (9×3)
//   - S block(i,i) -= E_k · W_obs[k]^T   ... wait, S is indexed by camera
//     pairs, not just diagonal. Actually:
//
// The full Schur complement in camera-camera blocks is:
//
//   S(i,i) = U[i] - sum_{k: cam(k)=i} E_k · W_obs[k]^T
//   S(i,j) = - sum_{k: cam(k)=i, l: cam(l)=j, pt(k)=pt(l)} E_k · W_obs[l]^T
//
// But this is complex. A cleaner way: iterate over points. For each point j,
// collect all observations that see it. Then:
//
//   V_j^{-1} is a 3×3 matrix
//   For each pair of observations (k, l) seeing point j:
//     S(cam(k), cam(l)) -= W_obs[k] · V_j^{-1} · W_obs[l]^T
//
// This is the standard "point elimination" approach.
//
// For the RHS:
//   For each observation k seeing point j:
//     rhs(cam(k)) -= W_obs[k] · V_j^{-1} · g_p[j]
//
// We start with S = block_diag(U) and rhs = g_c, then subtract.

void computeSchurComplement(const NormalEquations& neq, Eigen::MatrixXd& S,
                            Eigen::VectorXd& rhs) {
  int total_cam_dim = neq.num_cameras * CAM_DIM;
  S = Eigen::MatrixXd::Zero(total_cam_dim, total_cam_dim);
  rhs = Eigen::VectorXd::Zero(total_cam_dim);

  // Initialize S with block-diagonal U
  for (int i = 0; i < neq.num_cameras; ++i) {
    S.block<CAM_DIM, CAM_DIM>(i * CAM_DIM, i * CAM_DIM) = neq.U[i];
  }

  // Initialize rhs with g_c
  for (int i = 0; i < neq.num_cameras; ++i) {
    rhs.segment<CAM_DIM>(i * CAM_DIM) = neq.g_cameras[i];
  }

  // Build per-point observation lists
  // For each point j, collect the indices of observations that see it
  int num_obs = static_cast<int>(neq.W_obs.size());
  std::vector<std::vector<int>> point_observations(neq.num_points);
  for (int k = 0; k < num_obs; ++k) {
    point_observations[neq.obs_point_indices[k]].push_back(k);
  }

  // Precompute V_j^{-1} for each point
  std::vector<Eigen::Matrix<double, PT_DIM, PT_DIM>> V_inv(neq.num_points);
  for (int j = 0; j < neq.num_points; ++j) {
    // V[j] is 3×3 symmetric positive definite (sum of J^T J)
    // Use LLT for stable inversion
    V_inv[j] =
        neq.V[j].llt().solve(Eigen::Matrix<double, PT_DIM, PT_DIM>::Identity());
  }

  // For each point j, subtract the Schur contribution
  for (int j = 0; j < neq.num_points; ++j) {
    const auto& obs_list = point_observations[j];
    if (obs_list.empty()) continue;

    const auto& Vj_inv = V_inv[j];

    // Precompute E_k = W_obs[k] · V_j^{-1} for each observation seeing j
    // E_k is 9×3
    std::vector<Eigen::Matrix<double, CAM_DIM, PT_DIM>> E(obs_list.size());
    for (size_t idx = 0; idx < obs_list.size(); ++idx) {
      int k = obs_list[idx];
      E[idx].noalias() = neq.W_obs[k] * Vj_inv;
    }

    // S(cam(k), cam(l)) -= E_k · W_obs[l]^T
    for (size_t idx_k = 0; idx_k < obs_list.size(); ++idx_k) {
      int k = obs_list[idx_k];
      int cam_k = neq.obs_camera_indices[k];

      for (size_t idx_l = 0; idx_l < obs_list.size(); ++idx_l) {
        int l = obs_list[idx_l];
        int cam_l = neq.obs_camera_indices[l];

        // S(cam_k, cam_l) -= E_k · W_obs[l]^T   (9×3 · 3×9 = 9×9)
        S.block<CAM_DIM, CAM_DIM>(cam_k * CAM_DIM, cam_l * CAM_DIM).noalias() -=
            E[idx_k] * neq.W_obs[l].transpose();
      }

      // rhs(cam_k) -= E_k · g_p[j]   (9×3 · 3×1 = 9×1)
      rhs.segment<CAM_DIM>(cam_k * CAM_DIM).noalias() -=
          E[idx_k] * neq.g_points[j];
    }
  }
}

// =============================================================================
// Back-Substitution
// =============================================================================
//
// δp_j = -V_j^{-1} · (g_p_j + sum_{k seeing j} W_obs[k]^T · δc_{cam(k)})

void backSubstitute(const NormalEquations& neq,
                    const Eigen::VectorXd& delta_cameras,
                    Eigen::VectorXd& delta_points) {
  delta_points = Eigen::VectorXd::Zero(neq.num_points * PT_DIM);

  // Build per-point observation lists
  int num_obs = static_cast<int>(neq.W_obs.size());
  std::vector<std::vector<int>> point_observations(neq.num_points);
  for (int k = 0; k < num_obs; ++k) {
    point_observations[neq.obs_point_indices[k]].push_back(k);
  }

  for (int j = 0; j < neq.num_points; ++j) {
    // Accumulate W^T · δc for this point
    Eigen::Matrix<double, PT_DIM, 1> Wt_dc =
        Eigen::Matrix<double, PT_DIM, 1>::Zero();

    for (int k : point_observations[j]) {
      int cam_idx = neq.obs_camera_indices[k];
      // W_obs[k]^T · δc_{cam_idx}   (3×9 · 9×1 = 3×1)
      Wt_dc.noalias() += neq.W_obs[k].transpose() *
                         delta_cameras.segment<CAM_DIM>(cam_idx * CAM_DIM);
    }

    // δp_j = -V_j^{-1} · (g_p_j + Wt_dc)
    Eigen::Matrix<double, PT_DIM, 1> rhs_j = neq.g_points[j] + Wt_dc;
    delta_points.segment<PT_DIM>(j * PT_DIM) = -neq.V[j].llt().solve(rhs_j);
  }
}

}  // namespace solver
}  // namespace backend
}  // namespace substral
