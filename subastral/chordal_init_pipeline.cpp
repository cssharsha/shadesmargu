#include "subastral/chordal_init_pipeline.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

#include "subastral/backend/solver/lm_solver.hpp"
#include "subastral/backend/solver/pose_graph_solver.cuh"
#include "subastral/loader/g2o_loader.h"
#include "viz/rerun/pg_visualizer.hpp"

namespace substral {

using backend::Pose;
using backend::PoseEdge;
using backend::PoseGraphProblem;
using backend::solver::LMResult;

// =============================================================================
// Helpers
// =============================================================================

/// Convert quaternion (qx, qy, qz, qw) to 3x3 rotation matrix.
static Eigen::Matrix3d quatToRotation(double qx, double qy, double qz,
                                      double qw) {
  Eigen::Quaterniond q(qw, qx, qy, qz);
  q.normalize();
  return q.toRotationMatrix();
}

/// Convert 3x3 rotation matrix to quaternion (qx, qy, qz, qw).
static Eigen::Vector4d rotationToQuat(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond q(R);
  q.normalize();
  return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
}

/// Project a 3x3 matrix to the nearest rotation matrix via SVD.
static Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d& M) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
  // Ensure det(R) = +1 (not a reflection)
  if (R.determinant() < 0) {
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) *= -1;
    R = svd.matrixU() * V.transpose();
  }
  return R;
}

// =============================================================================
// Pipeline interface
// =============================================================================

bool ChordalInitPipeline::load(const std::string& filename) {
  loader::G2OLoader loader(filename);
  return loader.Load(problem_);
}

void ChordalInitPipeline::run(const PipelineConfig& config) {
  if (problem_.poses.empty()) {
    std::cerr << "No poses loaded. Nothing to optimize." << std::endl;
    return;
  }

  std::cout << std::string(60, '=') << std::endl;
  std::cout << "Subastral Pose-Graph SLAM (chordal init + GPU)" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "  Poses:          " << problem_.get_num_poses() << std::endl;
  std::cout << "  Edges:          " << problem_.get_num_edges() << std::endl;
  std::cout << "  Fixed vertex:   " << problem_.fixed_vertex_id << std::endl;
  std::cout << "  Max iterations: " << config.solver.max_iterations
            << std::endl;
  std::cout << "  Initial lambda: " << config.solver.initial_lambda
            << std::endl;
  std::cout << std::endl;

  // ---- Rerun visualization: log initial state ----
  rerun_viz::PGVisualizer rr_viz;
  if (config.rerun.enabled() && rr_viz.init(config.rerun)) {
    rr_viz.logPoses("initial", problem_);
    rr_viz.logEdges("initial", problem_);
    rr_viz.logTrajectory("initial", problem_);
  }

  // ---- Stage 1: Chordal rotation initialization ----
  std::cout << std::string(60, '-') << std::endl;
  std::cout << "Stage 1: Chordal rotation initialization..." << std::endl;
  std::cout << std::string(60, '-') << std::endl;

  auto t0 = std::chrono::high_resolution_clock::now();
  initializeRotations();
  auto t1 = std::chrono::high_resolution_clock::now();
  double rot_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  std::cout << "  Rotation init:  " << std::fixed << std::setprecision(1)
            << rot_ms << " ms" << std::defaultfloat << std::endl;

  // ---- Stage 2: Translation recovery ----
  std::cout << "Stage 2: Translation recovery..." << std::endl;

  auto t2 = std::chrono::high_resolution_clock::now();
  initializeTranslations();
  auto t3 = std::chrono::high_resolution_clock::now();
  double trans_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
  std::cout << "  Translation init: " << std::fixed << std::setprecision(1)
            << trans_ms << " ms" << std::defaultfloat << std::endl;

  // ---- Log initialized state ----
  if (config.rerun.enabled()) {
    rr_viz.logPoses("chordal_init", problem_);
    rr_viz.logEdges("chordal_init", problem_);
    rr_viz.logTrajectory("chordal_init", problem_);
  }

  // ---- Stage 3: Full pose-graph optimization ----
  // The chordal init provides a good enough starting point that we can
  // run the requested loss function directly — no L2 pre-pass needed.

  std::cout << std::string(60, '-') << std::endl;
  std::cout << "Stage 3: Pose-graph optimization..." << std::endl;
  std::cout << std::string(60, '-') << std::endl;

  auto start = std::chrono::high_resolution_clock::now();

  backend::solver::PoseGraphCallback callback = nullptr;
  if (config.rerun.enabled()) {
    callback = [&](int iter, double cost) -> bool {
      rr_viz.logIteration(iter, cost, config.solver.initial_lambda);
      return true;
    };
  }

  LMResult result =
      backend::solver::solvePoseGraph_GPU(problem_, config.solver, callback);

  auto end = std::chrono::high_resolution_clock::now();
  double solve_ms =
      std::chrono::duration<double, std::milli>(end - start).count();
  double total_ms = rot_ms + trans_ms + solve_ms;

  // ---- Results ----
  std::cout << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "Optimization complete" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "  Converged:      " << (result.converged ? "yes" : "NO")
            << std::endl;
  std::cout << "  Reason:         " << result.termination_reason << std::endl;
  std::cout << "  Iterations:     " << result.iterations << std::endl;
  std::cout << "  Initial cost:   " << result.initial_cost << std::endl;
  std::cout << "  Final cost:     " << result.final_cost << std::endl;
  std::cout << "  Cost reduction: " << std::fixed << std::setprecision(2)
            << (1.0 - result.final_cost / (result.initial_cost + 1e-30)) * 100.0
            << "%" << std::defaultfloat << std::endl;
  std::cout << "  Init time:      " << std::fixed << std::setprecision(1)
            << (rot_ms + trans_ms) << " ms (rot: " << rot_ms
            << " + trans: " << trans_ms << ")" << std::defaultfloat
            << std::endl;
  std::cout << "  Solve time:     " << std::fixed << std::setprecision(1)
            << solve_ms << " ms" << std::defaultfloat << std::endl;
  std::cout << "  Total time:     " << std::fixed << std::setprecision(1)
            << total_ms << " ms" << std::defaultfloat << std::endl;

  // ---- Rerun visualization: log final state ----
  if (config.rerun.enabled()) {
    rr_viz.logPoses("optimized", problem_);
    rr_viz.logEdges("optimized", problem_);
    rr_viz.logTrajectory("optimized", problem_);
  }
}

// =============================================================================
// Stage 1: Chordal rotation initialization
// =============================================================================
//
// Minimize sum_edges ||R_ij - R_i^T R_j||_F^2 using the chordal distance.
//
// Rewriting with rows of R_i:
//   Let r^k_i = k-th row of R_i (as a column vector), k = 1,2,3.
//   The unconstrained relaxation is:
//     min_{r^k_i} sum_{(i,j)} sum_{k=1,2,3} ||R_ij^T r^k_i - r^k_j||^2
//
// This is a linear least-squares problem. The 3 row systems share the same
// Hessian H but have different RHS. We solve H * X = RHS where X is 3n x 3.
//
// After solving, each 3x3 matrix M_i (rows from X) is projected to SO(3)
// via SVD: R_i = S * diag(1, 1, det(S V^T)) * V^T.
//
// The first pose is fixed as a prior to resolve gauge freedom.
//
void ChordalInitPipeline::initializeRotations() {
  int n = problem_.get_num_poses();
  int fixed_id = problem_.fixed_vertex_id;

  // Build sorted vertex ID → index mapping
  auto sorted_ids = problem_.sorted_vertex_ids();
  std::map<int, int> id_to_idx;
  for (int i = 0; i < n; ++i) {
    id_to_idx[sorted_ids[i]] = i;
  }

  // Find the fixed pose index
  int fixed_idx = 0;
  if (fixed_id >= 0) {
    fixed_idx = id_to_idx.at(fixed_id);
  }

  // Number of free rotations = n - 1 (one is fixed)
  int n_free = n - 1;

  // Map from pose index to free variable index (-1 if fixed)
  std::vector<int> var_map(n, -1);
  int var = 0;
  for (int i = 0; i < n; ++i) {
    if (i == fixed_idx) continue;
    var_map[i] = var++;
  }

  // Extract the fixed rotation
  Eigen::Matrix3d R_fixed;
  for (const auto& p : problem_.poses) {
    if (p->get_id() == sorted_ids[fixed_idx]) {
      R_fixed = quatToRotation(p->qx(), p->qy(), p->qz(), p->qw());
      break;
    }
  }

  // Build the sparse normal equations H * X = RHS.
  //
  // For each edge (i,j) with measurement R_ij, the residual for row k is:
  //   r^k = R_ij^T * r^k_i - r^k_j
  //
  // Jacobians: J_i = R_ij^T (3x3), J_j = -I (3x3)
  //
  // Normal equation contributions (unit weight, isotropic):
  //   H(vi,vi) += J_i^T J_i = R_ij R_ij^T = I
  //   H(vj,vj) += J_j^T J_j = I
  //   H(vi,vj) += J_i^T J_j = -R_ij
  //   H(vj,vi) += J_j^T J_i = -R_ij^T
  //
  // RHS columns correspond to the 3 row indices k=0,1,2.
  // X(vi*3 : vi*3+3, k) will be the k-th row of R_i (as a column vector).

  using Triplet = Eigen::Triplet<double>;
  std::vector<Triplet> triplets;
  triplets.reserve(problem_.get_num_edges() * 18);

  // RHS: 3*n_free rows x 3 columns (one per row of R)
  Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(3 * n_free, 3);

  for (const auto& edge : problem_.edges) {
    int idx_i = id_to_idx.at(edge->get_from_id());
    int idx_j = id_to_idx.at(edge->get_to_id());
    int vi = var_map[idx_i];
    int vj = var_map[idx_j];

    // Measured relative rotation R_ij
    const double* m = edge->measurement_data();
    Eigen::Matrix3d R_ij = quatToRotation(m[3], m[4], m[5], m[6]);

    if (vi >= 0 && vj >= 0) {
      // Both free
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          if (r == c) {
            triplets.push_back(Triplet(vi * 3 + r, vi * 3 + c, 1.0));
            triplets.push_back(Triplet(vj * 3 + r, vj * 3 + c, 1.0));
          }
          triplets.push_back(Triplet(vi * 3 + r, vj * 3 + c, -R_ij(r, c)));
          triplets.push_back(Triplet(vj * 3 + r, vi * 3 + c, -R_ij(c, r)));
        }
      }
      // RHS is zero for both-free edges
    } else if (vi >= 0 && vj < 0) {
      // j is fixed: residual = R_ij^T * r^k_i - r^k_fixed
      // Only r^k_i is variable. J = R_ij^T.
      // H(vi,vi) += I
      // RHS(vi, k) += R_ij * r^k_fixed = (R_ij * R_fixed^T)(row k as col)
      for (int r = 0; r < 3; ++r) {
        triplets.push_back(Triplet(vi * 3 + r, vi * 3 + r, 1.0));
      }
      // R_ij * R_fixed^T: column k of this matrix = R_ij * (k-th row of
      // R_fixed)
      Eigen::Matrix3d target = R_ij * R_fixed.transpose();
      for (int k = 0; k < 3; ++k) {
        for (int r = 0; r < 3; ++r) {
          rhs(vi * 3 + r, k) += target(r, k);
        }
      }
    } else if (vi < 0 && vj >= 0) {
      // i is fixed: residual = R_ij^T * r^k_fixed - r^k_j
      // Only r^k_j is variable. J = -I.
      // H(vj,vj) += I
      // RHS(vj, k) += R_ij^T * r^k_fixed = (R_ij^T * R_fixed^T)(col k)
      for (int r = 0; r < 3; ++r) {
        triplets.push_back(Triplet(vj * 3 + r, vj * 3 + r, 1.0));
      }
      Eigen::Matrix3d target = R_ij.transpose() * R_fixed.transpose();
      for (int k = 0; k < 3; ++k) {
        for (int r = 0; r < 3; ++r) {
          rhs(vj * 3 + r, k) += target(r, k);
        }
      }
    }
  }

  // Build sparse matrix and solve
  int sys_size = 3 * n_free;
  Eigen::SparseMatrix<double> H(sys_size, sys_size);
  H.setFromTriplets(triplets.begin(), triplets.end());

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  solver.compute(H);

  if (solver.info() != Eigen::Success) {
    std::cerr << "  WARNING: Chordal rotation solve failed (factorization)"
              << std::endl;
    return;
  }

  Eigen::MatrixXd X = solver.solve(rhs);

  if (solver.info() != Eigen::Success) {
    std::cerr << "  WARNING: Chordal rotation solve failed (back-substitution)"
              << std::endl;
    return;
  }

  // Extract and project rotations to SO(3) via SVD (eq. 23)
  for (int i = 0; i < n; ++i) {
    if (i == fixed_idx) continue;
    int vi = var_map[i];

    // X(vi*3 : vi*3+3, k) = k-th row of R_i (as column vector)
    // So M_i has rows from X columns: M_i^T has columns from X.
    // M_i(k, :) = X(vi*3 : vi*3+3, k)^T
    // => M_i = X_block^T
    Eigen::Matrix3d M;
    for (int k = 0; k < 3; ++k) {
      M.row(k) = X.block<3, 1>(vi * 3, k).transpose();
    }

    // Project to SO(3): R = S * diag(1, 1, det(SV^T)) * V^T
    Eigen::Matrix3d R = projectToSO3(M);
    Eigen::Vector4d q = rotationToQuat(R);

    // Update the pose's quaternion
    int id = sorted_ids[i];
    for (auto& p : problem_.poses) {
      if (p->get_id() == id) {
        double* data = p->mutable_data();
        data[3] = q[0];  // qx
        data[4] = q[1];  // qy
        data[5] = q[2];  // qz
        data[6] = q[3];  // qw
        break;
      }
    }
  }

  std::cout << "  Rotations initialized (" << n_free << " free poses)"
            << std::endl;
}

// =============================================================================
// Stage 2: Translation recovery
// =============================================================================
//
// With rotations R_i fixed, minimize:
//   sum_{(i,j)} ||t_j - t_i - R_i * t_ij||^2
//
// This is a sparse linear system in the translations t_i.
// Jacobians: J_i = -I, J_j = +I.
// RHS per edge: R_i * t_ij.
//
// Unit weights — anisotropic weights are applied in the subsequent
// nonlinear refinement.
//
void ChordalInitPipeline::initializeTranslations() {
  int n = problem_.get_num_poses();
  int fixed_id = problem_.fixed_vertex_id;

  auto sorted_ids = problem_.sorted_vertex_ids();
  std::map<int, int> id_to_idx;
  for (int i = 0; i < n; ++i) {
    id_to_idx[sorted_ids[i]] = i;
  }

  int fixed_idx = 0;
  if (fixed_id >= 0) {
    fixed_idx = id_to_idx.at(fixed_id);
  }

  int n_free = n - 1;
  std::vector<int> var_map(n, -1);
  int var = 0;
  for (int i = 0; i < n; ++i) {
    if (i == fixed_idx) continue;
    var_map[i] = var++;
  }

  // Extract current rotations and fixed translation
  std::vector<Eigen::Matrix3d> rotations(n);
  Eigen::Vector3d t_fixed;

  for (const auto& p : problem_.poses) {
    int idx = id_to_idx.at(p->get_id());
    rotations[idx] = quatToRotation(p->qx(), p->qy(), p->qz(), p->qw());
    if (idx == fixed_idx) {
      t_fixed = Eigen::Vector3d(p->x(), p->y(), p->z());
    }
  }

  // Build sparse linear system for: residual = t_j - t_i - R_i * t_ij
  // J_i = -I, J_j = +I
  // H(vi,vi) += I, H(vj,vj) += I, H(vi,vj) += -I, H(vj,vi) += -I
  // RHS_i += -(R_i * t_ij), RHS_j += R_i * t_ij
  using Triplet = Eigen::Triplet<double>;
  std::vector<Triplet> triplets;
  triplets.reserve(problem_.get_num_edges() * 12);

  int sys_size = 3 * n_free;
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(sys_size);

  for (const auto& edge : problem_.edges) {
    int idx_i = id_to_idx.at(edge->get_from_id());
    int idx_j = id_to_idx.at(edge->get_to_id());
    int vi = var_map[idx_i];
    int vj = var_map[idx_j];

    const Eigen::Matrix3d& Ri = rotations[idx_i];

    const double* m = edge->measurement_data();
    Eigen::Vector3d t_ij(m[0], m[1], m[2]);
    Eigen::Vector3d Ri_tij = Ri * t_ij;

    if (vi >= 0 && vj >= 0) {
      // Both free
      for (int r = 0; r < 3; ++r) {
        triplets.push_back(Triplet(vi * 3 + r, vi * 3 + r, 1.0));
        triplets.push_back(Triplet(vj * 3 + r, vj * 3 + r, 1.0));
        triplets.push_back(Triplet(vi * 3 + r, vj * 3 + r, -1.0));
        triplets.push_back(Triplet(vj * 3 + r, vi * 3 + r, -1.0));
      }
      for (int r = 0; r < 3; ++r) {
        rhs(vi * 3 + r) += -Ri_tij(r);
        rhs(vj * 3 + r) += Ri_tij(r);
      }
    } else if (vi >= 0 && vj < 0) {
      // j is fixed: residual = t_fixed - t_i - R_i * t_ij
      // J = -I, rhs = t_fixed - R_i * t_ij
      for (int r = 0; r < 3; ++r) {
        triplets.push_back(Triplet(vi * 3 + r, vi * 3 + r, 1.0));
      }
      Eigen::Vector3d contrib = t_fixed - Ri_tij;
      for (int r = 0; r < 3; ++r) {
        rhs(vi * 3 + r) += contrib(r);
      }
    } else if (vi < 0 && vj >= 0) {
      // i is fixed: residual = t_j - t_fixed - R_i * t_ij
      // J = +I, rhs = t_fixed + R_i * t_ij
      for (int r = 0; r < 3; ++r) {
        triplets.push_back(Triplet(vj * 3 + r, vj * 3 + r, 1.0));
      }
      Eigen::Vector3d contrib = t_fixed + Ri_tij;
      for (int r = 0; r < 3; ++r) {
        rhs(vj * 3 + r) += contrib(r);
      }
    }
  }

  // Solve
  Eigen::SparseMatrix<double> H(sys_size, sys_size);
  H.setFromTriplets(triplets.begin(), triplets.end());

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  solver.compute(H);

  if (solver.info() != Eigen::Success) {
    std::cerr << "  WARNING: Translation solve failed, keeping current "
                 "translations"
              << std::endl;
    return;
  }

  Eigen::VectorXd x = solver.solve(rhs);

  if (solver.info() != Eigen::Success) {
    std::cerr << "  WARNING: Translation solve failed, keeping current "
                 "translations"
              << std::endl;
    return;
  }

  // Write back translations
  for (int i = 0; i < n; ++i) {
    if (i == fixed_idx) continue;
    int vi = var_map[i];
    int id = sorted_ids[i];
    for (auto& p : problem_.poses) {
      if (p->get_id() == id) {
        double* data = p->mutable_data();
        data[0] = x(vi * 3 + 0);
        data[1] = x(vi * 3 + 1);
        data[2] = x(vi * 3 + 2);
        break;
      }
    }
  }

  std::cout << "  Translations initialized (" << n_free << " free poses)"
            << std::endl;
}

}  // namespace substral
