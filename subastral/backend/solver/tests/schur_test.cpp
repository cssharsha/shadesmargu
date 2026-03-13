#include "subastral/backend/solver/schur.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "subastral/backend/common.h"
#include "subastral/backend/ops/project_with_jacobian_cpu.hpp"

using namespace substral::backend;
using namespace substral::backend::solver;
using namespace substral::backend::ops;

// =============================================================================
// Test helper: build a small synthetic BA problem
// =============================================================================
//
// Creates num_cameras cameras and num_points points.
// Each camera observes all points (dense visibility).
// Camera params: small random angle-axis, random translation, f=500, k1=k2=0.
// Points: random in front of all cameras.
// Observations: project points through cameras (noise-free).
//
static BAProblem buildSyntheticProblem(int num_cameras, int num_points,
                                       unsigned seed = 42) {
  BAProblem problem;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> angle_dist(-0.1, 0.1);
  std::uniform_real_distribution<double> trans_dist(-1.0, 1.0);
  std::uniform_real_distribution<double> point_dist(-2.0, 2.0);
  std::uniform_real_distribution<double> depth_dist(5.0, 15.0);

  // Create cameras
  for (int i = 0; i < num_cameras; ++i) {
    auto cam = std::make_shared<Camera>(i, problem.memory_map);
    double* d = cam->mutable_data();
    // Small angle-axis rotation
    d[0] = angle_dist(rng);
    d[1] = angle_dist(rng);
    d[2] = angle_dist(rng);
    // Translation
    d[3] = trans_dist(rng);
    d[4] = trans_dist(rng);
    d[5] = trans_dist(rng);
    // Intrinsics
    d[6] = 500.0;  // focal length
    d[7] = 0.0;    // k1
    d[8] = 0.0;    // k2
    problem.cameras.push_back(cam);
  }

  // Create points (in front of all cameras, roughly along +Z)
  for (int j = 0; j < num_points; ++j) {
    auto pt = std::make_shared<Point>(j, problem.memory_map);
    double* d = pt->mutable_data();
    d[0] = point_dist(rng);
    d[1] = point_dist(rng);
    d[2] = depth_dist(rng);
    problem.points.push_back(pt);
  }

  // Create observations: each camera sees all points
  int obs_id = 0;
  for (int i = 0; i < num_cameras; ++i) {
    for (int j = 0; j < num_points; ++j) {
      auto obs =
          std::make_shared<Observation>(obs_id, i, j, problem.memory_map);
      // Project to get the observation
      const double* cam = problem.cameras[i]->data();
      const double* pt = problem.points[j]->data();
      double predicted[2];
      double J_cam[18], J_pt[6];
      projectWithJacobianCPU(cam, pt, predicted, J_cam, J_pt);

      // Store as observation data (noise-free)
      // Observations are stored in the memory map, need to set them directly
      double* obs_data =
          &problem.memory_map
               ->observations[problem.memory_map->observation_indices[obs_id]];
      obs_data[0] = predicted[0];
      obs_data[1] = predicted[1];

      problem.observations.push_back(obs);
      obs_id++;
    }
  }

  return problem;
}

// =============================================================================
// Helper: compute residuals and Jacobians for a problem
// =============================================================================
static void computeAllResidualsAndJacobians(const BAProblem& problem,
                                            std::vector<double>& residuals,
                                            std::vector<double>& J_cameras,
                                            std::vector<double>& J_points) {
  int num_obs = static_cast<int>(problem.observations.size());
  residuals.resize(num_obs * RES_DIM);
  J_cameras.resize(num_obs * RES_DIM * CAM_DIM);
  J_points.resize(num_obs * RES_DIM * PT_DIM);

  for (int k = 0; k < num_obs; ++k) {
    int cam_idx = problem.observations[k]->get_camera_id();
    int pt_idx = problem.observations[k]->get_point_id();
    const double* cam = problem.cameras[cam_idx]->data();
    const double* pt = problem.points[pt_idx]->data();

    double predicted[2];
    projectWithJacobianCPU(cam, pt, predicted,
                           &J_cameras[k * RES_DIM * CAM_DIM],
                           &J_points[k * RES_DIM * PT_DIM]);

    residuals[k * RES_DIM + 0] =
        predicted[0] - problem.observations[k]->data()[0];
    residuals[k * RES_DIM + 1] =
        predicted[1] - problem.observations[k]->data()[1];
  }
}

// =============================================================================
// Test 2f: Verify JtJ accumulation against brute-force dense J^T J
// =============================================================================
//
// Strategy:
//   1. Build a small problem (2 cameras, 3 points, 6 observations)
//   2. Compute per-observation Jacobians
//   3. Assemble the full dense Jacobian J (num_obs*2 ×
//   num_cameras*9+num_points*3)
//   4. Compute J^T J and J^T r directly
//   5. Compare block-by-block against the NormalEquations struct
//
TEST(SchurTest, JtJAccumulation_MatchesBruteForce) {
  constexpr int NC = 2;
  constexpr int NP = 3;
  BAProblem problem = buildSyntheticProblem(NC, NP);

  // Add some noise to points so residuals are nonzero
  std::mt19937 rng(123);
  std::normal_distribution<double> noise(0.0, 0.5);
  for (auto& pt : problem.points) {
    double* d = pt->mutable_data();
    d[0] += noise(rng);
    d[1] += noise(rng);
    d[2] += noise(rng);
  }

  int num_obs = static_cast<int>(problem.observations.size());
  ASSERT_EQ(num_obs, NC * NP);

  // Compute residuals and Jacobians
  std::vector<double> residuals, J_cameras, J_points;
  computeAllResidualsAndJacobians(problem, residuals, J_cameras, J_points);

  // ---- Block-sparse accumulation ----
  NormalEquations neq;
  accumulateNormalEquations(problem, residuals.data(), J_cameras.data(),
                            J_points.data(), neq);

  // ---- Brute-force: assemble full dense J ----
  // Variable ordering: [cam0(9), cam1(9), ..., pt0(3), pt1(3), ...]
  int total_cam_dim = NC * CAM_DIM;
  int total_pt_dim = NP * PT_DIM;
  int total_dim = total_cam_dim + total_pt_dim;
  int total_res = num_obs * RES_DIM;

  Eigen::MatrixXd J_full = Eigen::MatrixXd::Zero(total_res, total_dim);
  Eigen::VectorXd r_full(total_res);

  for (int k = 0; k < num_obs; ++k) {
    int cam_idx = problem.observations[k]->get_camera_id();
    int pt_idx = problem.observations[k]->get_point_id();

    Eigen::Map<const Eigen::Matrix<double, RES_DIM, CAM_DIM, Eigen::RowMajor>>
        J_ck(J_cameras.data() + k * RES_DIM * CAM_DIM);
    Eigen::Map<const Eigen::Matrix<double, RES_DIM, PT_DIM, Eigen::RowMajor>>
        J_pk(J_points.data() + k * RES_DIM * PT_DIM);

    J_full.block<RES_DIM, CAM_DIM>(k * RES_DIM, cam_idx * CAM_DIM) = J_ck;
    J_full.block<RES_DIM, PT_DIM>(k * RES_DIM,
                                  total_cam_dim + pt_idx * PT_DIM) = J_pk;

    r_full.segment<RES_DIM>(k * RES_DIM) =
        Eigen::Map<const Eigen::Vector2d>(residuals.data() + k * RES_DIM);
  }

  Eigen::MatrixXd JtJ_full = J_full.transpose() * J_full;
  Eigen::VectorXd Jtr_full = J_full.transpose() * r_full;

  // ---- Compare U blocks ----
  for (int i = 0; i < NC; ++i) {
    Eigen::MatrixXd U_expected =
        JtJ_full.block<CAM_DIM, CAM_DIM>(i * CAM_DIM, i * CAM_DIM);
    EXPECT_TRUE(neq.U[i].isApprox(U_expected, 1e-10))
        << "U[" << i << "] mismatch.\n"
        << "Expected:\n"
        << U_expected << "\nGot:\n"
        << neq.U[i];
  }

  // ---- Compare V blocks ----
  for (int j = 0; j < NP; ++j) {
    Eigen::MatrixXd V_expected = JtJ_full.block<PT_DIM, PT_DIM>(
        total_cam_dim + j * PT_DIM, total_cam_dim + j * PT_DIM);
    EXPECT_TRUE(neq.V[j].isApprox(V_expected, 1e-10))
        << "V[" << j << "] mismatch.\n"
        << "Expected:\n"
        << V_expected << "\nGot:\n"
        << neq.V[j];
  }

  // ---- Compare W blocks ----
  // W_obs[k] should equal J^T J block at (cam(k), total_cam_dim + pt(k))
  for (int k = 0; k < num_obs; ++k) {
    int cam_idx = neq.obs_camera_indices[k];
    int pt_idx = neq.obs_point_indices[k];

    // W_obs[k] = J_ck^T · J_pk, which is a single-observation contribution
    // to the (cam_idx, pt_idx) block of J^T J.
    // The full block is sum over all obs relating cam_idx to pt_idx.
    // In our dense problem each (cam, pt) pair has exactly one observation,
    // so W_obs[k] should equal the full block.
    Eigen::MatrixXd W_expected = JtJ_full.block<CAM_DIM, PT_DIM>(
        cam_idx * CAM_DIM, total_cam_dim + pt_idx * PT_DIM);
    EXPECT_TRUE(neq.W_obs[k].isApprox(W_expected, 1e-10))
        << "W_obs[" << k << "] (cam=" << cam_idx << ", pt=" << pt_idx
        << ") mismatch.\n"
        << "Expected:\n"
        << W_expected << "\nGot:\n"
        << neq.W_obs[k];
  }

  // ---- Compare gradient vectors ----
  for (int i = 0; i < NC; ++i) {
    Eigen::VectorXd gc_expected = Jtr_full.segment<CAM_DIM>(i * CAM_DIM);
    EXPECT_TRUE(neq.g_cameras[i].isApprox(gc_expected, 1e-10))
        << "g_cameras[" << i << "] mismatch.\n"
        << "Expected:\n"
        << gc_expected.transpose() << "\nGot:\n"
        << neq.g_cameras[i].transpose();
  }

  for (int j = 0; j < NP; ++j) {
    Eigen::VectorXd gp_expected =
        Jtr_full.segment<PT_DIM>(total_cam_dim + j * PT_DIM);
    EXPECT_TRUE(neq.g_points[j].isApprox(gp_expected, 1e-10))
        << "g_points[" << j << "] mismatch.\n"
        << "Expected:\n"
        << gp_expected.transpose() << "\nGot:\n"
        << neq.g_points[j].transpose();
  }
}

// =============================================================================
// Test 2g: Verify Schur complement solution matches direct dense solve
// =============================================================================
//
// Strategy:
//   1. Build the same small problem
//   2. Compute the block-sparse normal equations
//   3. Form the Schur complement S and rhs
//   4. Solve (S + λI)·δc = -rhs for δc, then back-substitute for δp
//   5. Also solve the full dense system (J^T J + λI)·δ = -J^T r directly
//   6. Compare δc and δp
//
// =============================================================================
// Test 2g: Verify Schur complement solution matches direct dense solve
// =============================================================================
//
// Strategy:
//   To get exact equivalence between Schur and dense solves, we must apply
//   LM damping to U and V blocks BEFORE forming the Schur complement.
//   The standard LM-with-Schur approach is:
//
//     S = (U + λ·diag(U)) - W·(V + λ·diag(V))^{-1}·W^T
//     rhs = g_c - W·(V + λ·diag(V))^{-1}·g_p
//     Solve: S·δc = -rhs
//     Back-sub: δp = -(V + λ·diag(V))^{-1}·(g_p + W^T·δc)
//
//   This is mathematically identical to solving the full damped system:
//     (J^T J + λ·diag(J^T J))·δ = -J^T r
//
//   We verify this by damping the NormalEquations blocks, forming the Schur
//   complement, solving, and comparing against the dense solve.
//
TEST(SchurTest, SchurSolution_MatchesDirectDenseSolve) {
  constexpr int NC = 2;
  constexpr int NP = 3;
  BAProblem problem = buildSyntheticProblem(NC, NP);

  // Add noise to points so residuals are nonzero
  std::mt19937 rng(456);
  std::normal_distribution<double> noise(0.0, 0.5);
  for (auto& pt : problem.points) {
    double* d = pt->mutable_data();
    d[0] += noise(rng);
    d[1] += noise(rng);
    d[2] += noise(rng);
  }

  int num_obs = static_cast<int>(problem.observations.size());

  // Compute residuals and Jacobians
  std::vector<double> residuals, J_cameras_vec, J_points_vec;
  computeAllResidualsAndJacobians(problem, residuals, J_cameras_vec,
                                  J_points_vec);

  // ---- Block-sparse path: Schur complement with pre-damped blocks ----
  NormalEquations neq;
  accumulateNormalEquations(problem, residuals.data(), J_cameras_vec.data(),
                            J_points_vec.data(), neq);

  // Apply LM damping to U and V blocks BEFORE forming Schur complement
  double lambda = 1e-3;
  NormalEquations neq_damped = neq;  // copy
  for (int i = 0; i < NC; ++i) {
    for (int d = 0; d < CAM_DIM; ++d) {
      neq_damped.U[i](d, d) += lambda * std::max(neq.U[i](d, d), 1e-6);
    }
  }
  for (int j = 0; j < NP; ++j) {
    for (int d = 0; d < PT_DIM; ++d) {
      neq_damped.V[j](d, d) += lambda * std::max(neq.V[j](d, d), 1e-6);
    }
  }

  Eigen::MatrixXd S;
  Eigen::VectorXd rhs;
  computeSchurComplement(neq_damped, S, rhs);

  // S is already damped (U was damped, V was damped), so solve directly
  Eigen::VectorXd delta_cameras_schur = S.ldlt().solve(-rhs);
  Eigen::VectorXd delta_points_schur;
  backSubstitute(neq_damped, delta_cameras_schur, delta_points_schur);

  // ---- Dense path: full system ----
  int total_cam_dim = NC * CAM_DIM;
  int total_pt_dim = NP * PT_DIM;
  int total_dim = total_cam_dim + total_pt_dim;
  int total_res = num_obs * RES_DIM;

  Eigen::MatrixXd J_full = Eigen::MatrixXd::Zero(total_res, total_dim);
  Eigen::VectorXd r_full(total_res);

  for (int k = 0; k < num_obs; ++k) {
    int cam_idx = problem.observations[k]->get_camera_id();
    int pt_idx = problem.observations[k]->get_point_id();

    Eigen::Map<const Eigen::Matrix<double, RES_DIM, CAM_DIM, Eigen::RowMajor>>
        J_ck(J_cameras_vec.data() + k * RES_DIM * CAM_DIM);
    Eigen::Map<const Eigen::Matrix<double, RES_DIM, PT_DIM, Eigen::RowMajor>>
        J_pk(J_points_vec.data() + k * RES_DIM * PT_DIM);

    J_full.block<RES_DIM, CAM_DIM>(k * RES_DIM, cam_idx * CAM_DIM) = J_ck;
    J_full.block<RES_DIM, PT_DIM>(k * RES_DIM,
                                  total_cam_dim + pt_idx * PT_DIM) = J_pk;

    r_full.segment<RES_DIM>(k * RES_DIM) =
        Eigen::Map<const Eigen::Vector2d>(residuals.data() + k * RES_DIM);
  }

  Eigen::MatrixXd JtJ_full = J_full.transpose() * J_full;
  Eigen::VectorXd Jtr_full = J_full.transpose() * r_full;

  // Apply same LM damping to full system
  Eigen::MatrixXd JtJ_damped = JtJ_full;
  for (int d = 0; d < total_dim; ++d) {
    JtJ_damped(d, d) += lambda * std::max(JtJ_full(d, d), 1e-6);
  }

  Eigen::VectorXd delta_full = JtJ_damped.ldlt().solve(-Jtr_full);
  Eigen::VectorXd delta_cameras_dense = delta_full.head(total_cam_dim);
  Eigen::VectorXd delta_points_dense = delta_full.tail(total_pt_dim);

  // ---- Compare (damped system is full-rank, solutions should match) ----
  EXPECT_TRUE(delta_cameras_schur.isApprox(delta_cameras_dense, 1e-8))
      << "Camera updates mismatch.\n"
      << "Schur:  " << delta_cameras_schur.transpose() << "\n"
      << "Dense:  " << delta_cameras_dense.transpose() << "\n"
      << "Diff norm: " << (delta_cameras_schur - delta_cameras_dense).norm();

  EXPECT_TRUE(delta_points_schur.isApprox(delta_points_dense, 1e-8))
      << "Point updates mismatch.\n"
      << "Schur:  " << delta_points_schur.transpose() << "\n"
      << "Dense:  " << delta_points_dense.transpose() << "\n"
      << "Diff norm: " << (delta_points_schur - delta_points_dense).norm();
}

// =============================================================================
// Test: Schur complement symmetry
// =============================================================================
//
// S should be symmetric since it's derived from J^T J which is symmetric.
//
TEST(SchurTest, SchurMatrix_IsSymmetric) {
  constexpr int NC = 3;
  constexpr int NP = 5;
  BAProblem problem = buildSyntheticProblem(NC, NP, 789);

  // Add noise
  std::mt19937 rng(321);
  std::normal_distribution<double> noise(0.0, 0.3);
  for (auto& pt : problem.points) {
    double* d = pt->mutable_data();
    d[0] += noise(rng);
    d[1] += noise(rng);
    d[2] += noise(rng);
  }

  std::vector<double> residuals, J_cameras, J_points;
  computeAllResidualsAndJacobians(problem, residuals, J_cameras, J_points);

  NormalEquations neq;
  accumulateNormalEquations(problem, residuals.data(), J_cameras.data(),
                            J_points.data(), neq);

  Eigen::MatrixXd S;
  Eigen::VectorXd rhs;
  computeSchurComplement(neq, S, rhs);

  // S should be symmetric (up to floating-point accumulation noise)
  // With many accumulated products, asymmetry ~1e-9 is expected.
  double asym = (S - S.transpose()).norm();
  double S_norm = S.norm();
  double relative_asym = asym / (S_norm + 1e-15);
  EXPECT_LT(relative_asym, 1e-12)
      << "S is not symmetric. Relative asymmetry = " << relative_asym
      << " (absolute = " << asym << ", ||S|| = " << S_norm << ")";

  // Symmetrize for eigenvalue check (standard practice)
  Eigen::MatrixXd S_sym = 0.5 * (S + S.transpose());

  // S should also be positive semi-definite (all eigenvalues >= 0)
  // BA has gauge freedom (7 DOF), so some eigenvalues may be near zero.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(S_sym);
  double min_eigenvalue = eig.eigenvalues().minCoeff();
  EXPECT_GE(min_eigenvalue, -1e-8)
      << "S has negative eigenvalue: " << min_eigenvalue;
}
