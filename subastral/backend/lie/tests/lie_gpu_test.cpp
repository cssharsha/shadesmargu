#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

#include "subastral/backend/lie/lie_gpu_test_kernels.cuh"
#include "subastral/backend/lie/se3.hpp"
#include "subastral/backend/lie/so3.hpp"

namespace substral {
namespace backend {
namespace lie {
namespace {

constexpr double kTol = 1e-10;
constexpr int N = 200;  // Number of test cases per operation

// =============================================================================
// CUDA helper
// =============================================================================

#define CUDA_CHECK(call)                                                      \
  do {                                                                        \
    cudaError_t err = call;                                                   \
    ASSERT_EQ(err, cudaSuccess) << "CUDA error: " << cudaGetErrorString(err); \
  } while (0)

// =============================================================================
// Random generators
// =============================================================================

Eigen::Vector3d randomAngleAxis(std::mt19937 &rng, double max_angle) {
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  Eigen::Vector3d w(dist(rng), dist(rng), dist(rng));
  if (w.norm() < 1e-12) w = Eigen::Vector3d(1, 0, 0);
  w.normalize();
  std::uniform_real_distribution<double> angle_dist(0.01, max_angle);
  return w * angle_dist(rng);
}

Eigen::Matrix<double, 6, 1> randomTwist(std::mt19937 &rng) {
  Eigen::Vector3d w = randomAngleAxis(rng, M_PI - 0.01);
  std::uniform_real_distribution<double> dist(-5.0, 5.0);
  Eigen::Vector3d v(dist(rng), dist(rng), dist(rng));
  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = w;
  xi.tail<3>() = v;
  return xi;
}

// =============================================================================
// SO(3) exp: GPU vs CPU
// =============================================================================

TEST(LieGPU, ExpSO3MatchesCPU) {
  std::mt19937 rng(42);

  // Generate random angle-axis vectors
  std::vector<double> h_w(N * 3);
  std::vector<double> h_R_cpu(N * 9);
  std::vector<double> h_R_gpu(N * 9);

  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d w = randomAngleAxis(rng, M_PI - 0.01);
    for (int j = 0; j < 3; ++j) h_w[i * 3 + j] = w(j);

    // CPU reference
    Eigen::Matrix3d R = expSO3(w);
    // Store row-major
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) h_R_cpu[i * 9 + r * 3 + c] = R(r, c);
  }

  // GPU
  double *d_w, *d_R;
  CUDA_CHECK(cudaMalloc(&d_w, N * 3 * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_R, N * 9 * sizeof(double)));
  CUDA_CHECK(cudaMemcpy(d_w, h_w.data(), N * 3 * sizeof(double),
                        cudaMemcpyHostToDevice));

  launchExpSO3Test(d_w, d_R, N);
  CUDA_CHECK(cudaDeviceSynchronize());

  CUDA_CHECK(cudaMemcpy(h_R_gpu.data(), d_R, N * 9 * sizeof(double),
                        cudaMemcpyDeviceToHost));

  // Compare
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < 9; ++j) {
      EXPECT_NEAR(h_R_cpu[i * 9 + j], h_R_gpu[i * 9 + j], kTol)
          << "Mismatch at element " << j << " of rotation " << i;
    }
  }

  cudaFree(d_w);
  cudaFree(d_R);
}

// =============================================================================
// SO(3) log: GPU vs CPU
// =============================================================================

TEST(LieGPU, LogSO3MatchesCPU) {
  std::mt19937 rng(123);

  std::vector<double> h_R(N * 9);
  std::vector<double> h_w_cpu(N * 3);
  std::vector<double> h_w_gpu(N * 3);

  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d w_orig = randomAngleAxis(rng, M_PI - 0.01);
    Eigen::Matrix3d R = expSO3(w_orig);

    // Store R row-major
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) h_R[i * 9 + r * 3 + c] = R(r, c);

    // CPU reference
    Eigen::Vector3d w = logSO3(R);
    for (int j = 0; j < 3; ++j) h_w_cpu[i * 3 + j] = w(j);
  }

  double *d_R, *d_w;
  CUDA_CHECK(cudaMalloc(&d_R, N * 9 * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_w, N * 3 * sizeof(double)));
  CUDA_CHECK(cudaMemcpy(d_R, h_R.data(), N * 9 * sizeof(double),
                        cudaMemcpyHostToDevice));

  launchLogSO3Test(d_R, d_w, N);
  CUDA_CHECK(cudaDeviceSynchronize());

  CUDA_CHECK(cudaMemcpy(h_w_gpu.data(), d_w, N * 3 * sizeof(double),
                        cudaMemcpyDeviceToHost));

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(h_w_cpu[i * 3 + j], h_w_gpu[i * 3 + j], kTol)
          << "Mismatch at element " << j << " of log " << i;
    }
  }

  cudaFree(d_R);
  cudaFree(d_w);
}

// =============================================================================
// SO(3) Left Jacobian: GPU vs CPU
// =============================================================================

TEST(LieGPU, LeftJacobianSO3MatchesCPU) {
  std::mt19937 rng(456);

  std::vector<double> h_w(N * 3);
  std::vector<double> h_Jl_cpu(N * 9);
  std::vector<double> h_Jl_gpu(N * 9);

  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d w = randomAngleAxis(rng, M_PI - 0.01);
    for (int j = 0; j < 3; ++j) h_w[i * 3 + j] = w(j);

    Eigen::Matrix3d Jl = leftJacobianSO3(w);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) h_Jl_cpu[i * 9 + r * 3 + c] = Jl(r, c);
  }

  double *d_w, *d_Jl;
  CUDA_CHECK(cudaMalloc(&d_w, N * 3 * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_Jl, N * 9 * sizeof(double)));
  CUDA_CHECK(cudaMemcpy(d_w, h_w.data(), N * 3 * sizeof(double),
                        cudaMemcpyHostToDevice));

  launchLeftJacobianSO3Test(d_w, d_Jl, N);
  CUDA_CHECK(cudaDeviceSynchronize());

  CUDA_CHECK(cudaMemcpy(h_Jl_gpu.data(), d_Jl, N * 9 * sizeof(double),
                        cudaMemcpyDeviceToHost));

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < 9; ++j) {
      EXPECT_NEAR(h_Jl_cpu[i * 9 + j], h_Jl_gpu[i * 9 + j], kTol)
          << "Mismatch at element " << j << " of Jl " << i;
    }
  }

  cudaFree(d_w);
  cudaFree(d_Jl);
}

// =============================================================================
// SO(3) Left Jacobian Inverse: GPU vs CPU
// =============================================================================

TEST(LieGPU, LeftJacobianInvSO3MatchesCPU) {
  std::mt19937 rng(789);

  std::vector<double> h_w(N * 3);
  std::vector<double> h_Jli_cpu(N * 9);
  std::vector<double> h_Jli_gpu(N * 9);

  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d w = randomAngleAxis(rng, M_PI - 0.01);
    for (int j = 0; j < 3; ++j) h_w[i * 3 + j] = w(j);

    Eigen::Matrix3d Jli = leftJacobianInverseSO3(w);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) h_Jli_cpu[i * 9 + r * 3 + c] = Jli(r, c);
  }

  double *d_w, *d_Jli;
  CUDA_CHECK(cudaMalloc(&d_w, N * 3 * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_Jli, N * 9 * sizeof(double)));
  CUDA_CHECK(cudaMemcpy(d_w, h_w.data(), N * 3 * sizeof(double),
                        cudaMemcpyHostToDevice));

  launchLeftJacobianInvSO3Test(d_w, d_Jli, N);
  CUDA_CHECK(cudaDeviceSynchronize());

  CUDA_CHECK(cudaMemcpy(h_Jli_gpu.data(), d_Jli, N * 9 * sizeof(double),
                        cudaMemcpyDeviceToHost));

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < 9; ++j) {
      EXPECT_NEAR(h_Jli_cpu[i * 9 + j], h_Jli_gpu[i * 9 + j], kTol)
          << "Mismatch at element " << j << " of Jl_inv " << i;
    }
  }

  cudaFree(d_w);
  cudaFree(d_Jli);
}

// =============================================================================
// SE(3) exp: GPU vs CPU
// =============================================================================

TEST(LieGPU, ExpSE3MatchesCPU) {
  std::mt19937 rng(111);

  std::vector<double> h_xi(N * 6);
  std::vector<double> h_R_cpu(N * 9);
  std::vector<double> h_t_cpu(N * 3);
  std::vector<double> h_R_gpu(N * 9);
  std::vector<double> h_t_gpu(N * 3);

  for (int i = 0; i < N; ++i) {
    Eigen::Matrix<double, 6, 1> xi = randomTwist(rng);
    for (int j = 0; j < 6; ++j) h_xi[i * 6 + j] = xi(j);

    Eigen::Matrix4d T = expSE3(xi);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) h_R_cpu[i * 9 + r * 3 + c] = T(r, c);
    for (int j = 0; j < 3; ++j) h_t_cpu[i * 3 + j] = T(j, 3);
  }

  double *d_xi, *d_R, *d_t;
  CUDA_CHECK(cudaMalloc(&d_xi, N * 6 * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_R, N * 9 * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_t, N * 3 * sizeof(double)));
  CUDA_CHECK(cudaMemcpy(d_xi, h_xi.data(), N * 6 * sizeof(double),
                        cudaMemcpyHostToDevice));

  launchExpSE3Test(d_xi, d_R, d_t, N);
  CUDA_CHECK(cudaDeviceSynchronize());

  CUDA_CHECK(cudaMemcpy(h_R_gpu.data(), d_R, N * 9 * sizeof(double),
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(h_t_gpu.data(), d_t, N * 3 * sizeof(double),
                        cudaMemcpyDeviceToHost));

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < 9; ++j) {
      EXPECT_NEAR(h_R_cpu[i * 9 + j], h_R_gpu[i * 9 + j], kTol)
          << "R mismatch at element " << j << " of SE3 exp " << i;
    }
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(h_t_cpu[i * 3 + j], h_t_gpu[i * 3 + j], kTol)
          << "t mismatch at element " << j << " of SE3 exp " << i;
    }
  }

  cudaFree(d_xi);
  cudaFree(d_R);
  cudaFree(d_t);
}

// =============================================================================
// SE(3) log: GPU vs CPU
// =============================================================================

TEST(LieGPU, LogSE3MatchesCPU) {
  std::mt19937 rng(222);

  std::vector<double> h_R(N * 9);
  std::vector<double> h_t(N * 3);
  std::vector<double> h_xi_cpu(N * 6);
  std::vector<double> h_xi_gpu(N * 6);

  for (int i = 0; i < N; ++i) {
    // Generate a valid SE(3) element via exp
    Eigen::Matrix<double, 6, 1> xi_orig = randomTwist(rng);
    Eigen::Matrix4d T = expSE3(xi_orig);

    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) h_R[i * 9 + r * 3 + c] = T(r, c);
    for (int j = 0; j < 3; ++j) h_t[i * 3 + j] = T(j, 3);

    // CPU reference
    Eigen::Matrix<double, 6, 1> xi = logSE3(T);
    for (int j = 0; j < 6; ++j) h_xi_cpu[i * 6 + j] = xi(j);
  }

  double *d_R, *d_t, *d_xi;
  CUDA_CHECK(cudaMalloc(&d_R, N * 9 * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_t, N * 3 * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_xi, N * 6 * sizeof(double)));
  CUDA_CHECK(cudaMemcpy(d_R, h_R.data(), N * 9 * sizeof(double),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_t, h_t.data(), N * 3 * sizeof(double),
                        cudaMemcpyHostToDevice));

  launchLogSE3Test(d_R, d_t, d_xi, N);
  CUDA_CHECK(cudaDeviceSynchronize());

  CUDA_CHECK(cudaMemcpy(h_xi_gpu.data(), d_xi, N * 6 * sizeof(double),
                        cudaMemcpyDeviceToHost));

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(h_xi_cpu[i * 6 + j], h_xi_gpu[i * 6 + j], kTol)
          << "xi mismatch at element " << j << " of SE3 log " << i;
    }
  }

  cudaFree(d_R);
  cudaFree(d_t);
  cudaFree(d_xi);
}

// =============================================================================
// SE(3) transform point: GPU vs CPU
// =============================================================================

TEST(LieGPU, TransformPointMatchesCPU) {
  std::mt19937 rng(333);

  std::vector<double> h_R(N * 9);
  std::vector<double> h_t(N * 3);
  std::vector<double> h_p(N * 3);
  std::vector<double> h_pout_cpu(N * 3);
  std::vector<double> h_pout_gpu(N * 3);

  std::uniform_real_distribution<double> pdist(-10.0, 10.0);

  for (int i = 0; i < N; ++i) {
    Eigen::Matrix<double, 6, 1> xi = randomTwist(rng);
    Eigen::Matrix4d T = expSE3(xi);

    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) h_R[i * 9 + r * 3 + c] = T(r, c);
    for (int j = 0; j < 3; ++j) h_t[i * 3 + j] = T(j, 3);

    Eigen::Vector3d p(pdist(rng), pdist(rng), pdist(rng));
    for (int j = 0; j < 3; ++j) h_p[i * 3 + j] = p(j);

    // CPU reference
    Eigen::Vector3d p_out = transformPoint(T, p);
    for (int j = 0; j < 3; ++j) h_pout_cpu[i * 3 + j] = p_out(j);
  }

  double *d_R, *d_t, *d_p, *d_pout;
  CUDA_CHECK(cudaMalloc(&d_R, N * 9 * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_t, N * 3 * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_p, N * 3 * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_pout, N * 3 * sizeof(double)));
  CUDA_CHECK(cudaMemcpy(d_R, h_R.data(), N * 9 * sizeof(double),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_t, h_t.data(), N * 3 * sizeof(double),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_p, h_p.data(), N * 3 * sizeof(double),
                        cudaMemcpyHostToDevice));

  launchTransformPointTest(d_R, d_t, d_p, d_pout, N);
  CUDA_CHECK(cudaDeviceSynchronize());

  CUDA_CHECK(cudaMemcpy(h_pout_gpu.data(), d_pout, N * 3 * sizeof(double),
                        cudaMemcpyDeviceToHost));

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(h_pout_cpu[i * 3 + j], h_pout_gpu[i * 3 + j], kTol)
          << "p_out mismatch at element " << j << " of transform " << i;
    }
  }

  cudaFree(d_R);
  cudaFree(d_t);
  cudaFree(d_p);
  cudaFree(d_pout);
}

}  // namespace
}  // namespace lie
}  // namespace backend
}  // namespace substral
