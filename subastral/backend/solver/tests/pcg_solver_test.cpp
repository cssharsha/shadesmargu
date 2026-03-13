#include "subastral/backend/solver/pcg_solver.cuh"

#include <cuda_runtime.h>

#include <cmath>
#include <iostream>
#include <vector>

#include "gtest/gtest.h"
#include "subastral/backend/common.h"
#include "subastral/backend/solver/pose_graph_kernels.cuh"
#include "subastral/backend/solver/pose_graph_solver.cuh"
#include "subastral/loader/g2o_loader.h"

namespace substral {
namespace backend {
namespace solver {
namespace {

// =============================================================================
// Helper: RAII wrapper for device memory (avoids thrust in .cpp files)
// =============================================================================
class DeviceBuffer {
 public:
  DeviceBuffer() : ptr_(nullptr), size_(0) {}
  explicit DeviceBuffer(size_t bytes) : size_(bytes) {
    cudaMalloc(&ptr_, bytes);
  }
  ~DeviceBuffer() {
    if (ptr_) cudaFree(ptr_);
  }

  // Upload from host
  void upload(const void* host_data, size_t bytes) {
    cudaMemcpy(ptr_, host_data, bytes, cudaMemcpyHostToDevice);
  }
  void upload(const void* host_data) { upload(host_data, size_); }

  // Download to host
  void download(void* host_data, size_t bytes) const {
    cudaMemcpy(host_data, ptr_, bytes, cudaMemcpyDeviceToHost);
  }
  void download(void* host_data) const { download(host_data, size_); }

  // Zero the buffer
  void zero() { cudaMemset(ptr_, 0, size_); }

  template <typename T>
  T* as() {
    return static_cast<T*>(ptr_);
  }
  template <typename T>
  const T* as() const {
    return static_cast<const T*>(ptr_);
  }

  DeviceBuffer(DeviceBuffer&& other) noexcept
      : ptr_(other.ptr_), size_(other.size_) {
    other.ptr_ = nullptr;
    other.size_ = 0;
  }
  DeviceBuffer& operator=(DeviceBuffer&& other) noexcept {
    if (this != &other) {
      if (ptr_) cudaFree(ptr_);
      ptr_ = other.ptr_;
      size_ = other.size_;
      other.ptr_ = nullptr;
      other.size_ = 0;
    }
    return *this;
  }

  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;

 private:
  void* ptr_;
  size_t size_;
};

// Helper: create device buffer from host vector
template <typename T>
DeviceBuffer makeDevice(const std::vector<T>& h) {
  DeviceBuffer buf(h.size() * sizeof(T));
  buf.upload(h.data());
  return buf;
}

// =============================================================================
// Test 1: Symmetric SpMV correctness
// =============================================================================
TEST(PCGSolverTest, SymmetricSpMV) {
  // 6x6 tridiagonal SPD matrix (lower triangle CSR):
  //   [ 4  1  0  0  0  0 ]
  //   [ 1  4  1  0  0  0 ]
  //   [ 0  1  4  1  0  0 ]
  //   [ 0  0  1  4  1  0 ]
  //   [ 0  0  0  1  4  1 ]
  //   [ 0  0  0  0  1  4 ]
  int n = 6;
  std::vector<int> row_ptr = {0, 1, 3, 5, 7, 9, 11};
  std::vector<int> col_ind = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5};
  std::vector<double> values = {4.0, 1.0, 4.0, 1.0, 4.0, 1.0,
                                4.0, 1.0, 4.0, 1.0, 4.0};
  std::vector<double> x = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  std::vector<double> expected = {6.0, 12.0, 18.0, 24.0, 30.0, 29.0};

  auto d_row_ptr = makeDevice(row_ptr);
  auto d_col_ind = makeDevice(col_ind);
  auto d_values = makeDevice(values);
  auto d_x = makeDevice(x);
  DeviceBuffer d_y(n * sizeof(double));
  d_y.zero();

  pg_gpu::launchSymmetricSpMV(d_row_ptr.as<int>(), d_col_ind.as<int>(),
                              d_values.as<double>(), d_x.as<double>(),
                              d_y.as<double>(), n);
  cudaDeviceSynchronize();

  std::vector<double> h_y(n);
  d_y.download(h_y.data());

  for (int i = 0; i < n; ++i) {
    EXPECT_NEAR(h_y[i], expected[i], 1e-12) << "SpMV mismatch at index " << i;
  }
}

// =============================================================================
// Test 2: Block-diagonal preconditioner correctness
// =============================================================================
TEST(PCGSolverTest, BlockDiagonalPreconditioner) {
  int n = 6;

  // Lower triangle CSR of a diagonal matrix diag(2,3,4,5,6,7)
  std::vector<int> row_ptr(n + 1);
  std::vector<int> col_ind_vec;
  std::vector<double> values_vec;

  int nnz = 0;
  for (int r = 0; r < n; ++r) {
    row_ptr[r] = nnz;
    for (int c = 0; c <= r; ++c) {
      col_ind_vec.push_back(c);
      values_vec.push_back((c == r) ? (2.0 + r) : 0.0);
      ++nnz;
    }
  }
  row_ptr[n] = nnz;

  // diag_offsets: for row r, the CSR value index where the diagonal block
  // starts
  std::vector<int> diag_offsets(n);
  for (int r = 0; r < n; ++r) {
    diag_offsets[r] = row_ptr[r];
  }

  auto d_values = makeDevice(values_vec);
  auto d_diag_offsets = makeDevice(diag_offsets);
  DeviceBuffer d_inv_blocks(36 * sizeof(double));
  d_inv_blocks.zero();

  pg_gpu::launchExtractAndInvertDiagBlocks(d_values.as<double>(),
                                           d_diag_offsets.as<int>(), 1,
                                           d_inv_blocks.as<double>());
  cudaDeviceSynchronize();

  std::vector<double> h_inv(36);
  d_inv_blocks.download(h_inv.data());

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double expected = (i == j) ? 1.0 / (2.0 + i) : 0.0;
      EXPECT_NEAR(h_inv[i * 6 + j], expected, 1e-10)
          << "Preconditioner inverse mismatch at (" << i << "," << j << ")";
    }
  }

  // Test apply: z = M^{-1} * r where r = (2,3,4,5,6,7) -> z = (1,1,1,1,1,1)
  std::vector<double> r_vec = {2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  auto d_r = makeDevice(r_vec);
  DeviceBuffer d_z(6 * sizeof(double));
  d_z.zero();

  pg_gpu::launchApplyBlockPreconditioner(d_inv_blocks.as<double>(),
                                         d_r.as<double>(), 1, d_z.as<double>());
  cudaDeviceSynchronize();

  std::vector<double> h_z(6);
  d_z.download(h_z.data());

  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(h_z[i], 1.0, 1e-10) << "Preconditioner apply mismatch at " << i;
  }
}

// =============================================================================
// Test 3: PCG on a known SPD system
// =============================================================================
TEST(PCGSolverTest, PCGSolvesKnownSPDSystem) {
  int n = 6;
  std::vector<int> row_ptr = {0, 1, 3, 5, 7, 9, 11};
  std::vector<int> col_ind = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5};
  std::vector<double> values = {4.0, 1.0, 4.0, 1.0, 4.0, 1.0,
                                4.0, 1.0, 4.0, 1.0, 4.0};

  // RHS chosen so that x = (1, 2, 3, 4, 5, 6)
  std::vector<double> b = {6.0, 12.0, 18.0, 24.0, 30.0, 29.0};

  auto d_row_ptr = makeDevice(row_ptr);
  auto d_col_ind = makeDevice(col_ind);
  auto d_values = makeDevice(values);
  auto d_b = makeDevice(b);
  DeviceBuffer d_x(n * sizeof(double));
  d_x.zero();

  // Scalar Jacobi preconditioner: diag = 4.0 everywhere
  std::vector<double> inv_block(36, 0.0);
  for (int i = 0; i < 6; ++i) {
    inv_block[i * 6 + i] = 1.0 / 4.0;
  }
  auto d_precond = makeDevice(inv_block);

  // Workspace
  DeviceBuffer d_r(n * sizeof(double));
  DeviceBuffer d_z(n * sizeof(double));
  DeviceBuffer d_p(n * sizeof(double));
  DeviceBuffer d_Ap(n * sizeof(double));

  int iters =
      solvePCG_GPU(d_row_ptr.as<int>(), d_col_ind.as<int>(),
                   d_values.as<double>(), d_b.as<double>(), d_x.as<double>(),
                   d_precond.as<double>(), n, 1,  // num_free_poses = 1
                   500, 1e-10, d_r.as<double>(), d_z.as<double>(),
                   d_p.as<double>(), d_Ap.as<double>());

  EXPECT_GT(iters, 0) << "PCG should converge";

  std::vector<double> h_x(n);
  d_x.download(h_x.data());

  std::vector<double> expected = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  for (int i = 0; i < n; ++i) {
    EXPECT_NEAR(h_x[i], expected[i], 1e-6)
        << "PCG solution mismatch at index " << i;
  }

  std::cout << "PCG converged in " << iters << " iterations" << std::endl;
}

// =============================================================================
// Test 4: Vector operations (axpy, xpby, negate)
// =============================================================================
TEST(PCGSolverTest, VectorOperations) {
  int n = 8;
  std::vector<double> x_h = {1, 2, 3, 4, 5, 6, 7, 8};
  std::vector<double> y_h = {10, 20, 30, 40, 50, 60, 70, 80};

  auto d_x = makeDevice(x_h);
  auto d_y = makeDevice(y_h);

  // Test axpy: y = 2.0 * x + y
  pg_gpu::launchAxpy(2.0, d_x.as<double>(), d_y.as<double>(), n);
  cudaDeviceSynchronize();

  std::vector<double> result(n);
  d_y.download(result.data());
  for (int i = 0; i < n; ++i) {
    EXPECT_NEAR(result[i], y_h[i] + 2.0 * x_h[i], 1e-12);
  }

  // Reset y
  d_y.upload(y_h.data());

  // Test xpby: y = x + 0.5 * y
  pg_gpu::launchXpby(d_x.as<double>(), 0.5, d_y.as<double>(), n);
  cudaDeviceSynchronize();

  d_y.download(result.data());
  for (int i = 0; i < n; ++i) {
    EXPECT_NEAR(result[i], x_h[i] + 0.5 * y_h[i], 1e-12);
  }

  // Test negate: neg = -x
  DeviceBuffer d_neg(n * sizeof(double));
  pg_gpu::launchNegate(d_x.as<double>(), d_neg.as<double>(), n);
  cudaDeviceSynchronize();

  d_neg.download(result.data());
  for (int i = 0; i < n; ++i) {
    EXPECT_NEAR(result[i], -x_h[i], 1e-12);
  }
}

// =============================================================================
// Test 5: Pose-graph with PCG — torus3D
// =============================================================================
TEST(PCGSolverTest, Torus3D_PCG) {
  loader::G2OLoader loader("/data/se/g2o_datasets/torus3D.g2o");
  PoseGraphProblem problem;
  ASSERT_TRUE(loader.Load(problem));

  std::cout << "Loaded: " << problem.get_num_poses() << " poses, "
            << problem.get_num_edges() << " edges" << std::endl;

  LMConfig config;
  config.max_iterations = 30;
  config.verbose = true;
  config.initial_lambda = 1e-3;
  config.cost_tolerance = 1e-4;
  config.linear_solver = LinearSolverType::PCG;
  config.pcg_max_iterations = 500;
  config.pcg_tolerance = 1e-8;

  LMResult result = solvePoseGraph_GPU(problem, config);

  std::cout << "PCG Result: " << result.termination_reason << std::endl;
  std::cout << "  Initial cost: " << result.initial_cost << std::endl;
  std::cout << "  Final cost:   " << result.final_cost << std::endl;
  std::cout << "  Iterations:   " << result.iterations << std::endl;
  std::cout << "  Cost ratio:   " << result.final_cost / result.initial_cost
            << std::endl;

  EXPECT_LT(result.final_cost, result.initial_cost);
  EXPECT_LT(result.final_cost, result.initial_cost * 0.1);
}

// =============================================================================
// Test 6: Pose-graph with PCG — parking-garage
// =============================================================================
TEST(PCGSolverTest, ParkingGarage_PCG) {
  loader::G2OLoader loader("/data/se/g2o_datasets/parking-garage.g2o");
  PoseGraphProblem problem;
  ASSERT_TRUE(loader.Load(problem));

  LMConfig config;
  config.max_iterations = 30;
  config.verbose = true;
  config.initial_lambda = 1e-3;
  config.linear_solver = LinearSolverType::PCG;
  config.pcg_max_iterations = 500;
  config.pcg_tolerance = 1e-8;

  LMResult result = solvePoseGraph_GPU(problem, config);

  std::cout << "parking-garage PCG result: " << result.termination_reason
            << std::endl;
  std::cout << "  Initial cost: " << result.initial_cost << std::endl;
  std::cout << "  Final cost:   " << result.final_cost << std::endl;
  std::cout << "  Cost ratio:   " << result.final_cost / result.initial_cost
            << std::endl;

  EXPECT_LT(result.final_cost, result.initial_cost);
}

// =============================================================================
// Test 7: Pose-graph with PCG — cubicle (previously failed with Cholesky)
// =============================================================================
TEST(PCGSolverTest, Cubicle_PCG) {
  loader::G2OLoader loader("/data/se/g2o_datasets/cubicle.g2o");
  PoseGraphProblem problem;
  ASSERT_TRUE(loader.Load(problem));

  std::cout << "cubicle: " << problem.get_num_poses() << " poses, "
            << problem.get_num_edges() << " edges" << std::endl;

  LMConfig config;
  config.max_iterations = 50;
  config.verbose = true;
  config.initial_lambda = 1.0;
  config.linear_solver = LinearSolverType::PCG;
  config.pcg_max_iterations = 1000;
  config.pcg_tolerance = 1e-6;

  LMResult result = solvePoseGraph_GPU(problem, config);

  std::cout << "cubicle PCG result: " << result.termination_reason << std::endl;
  std::cout << "  Initial cost: " << result.initial_cost << std::endl;
  std::cout << "  Final cost:   " << result.final_cost << std::endl;
  std::cout << "  Cost ratio:   " << result.final_cost / result.initial_cost
            << std::endl;

  // The key test: cubicle should now make progress
  EXPECT_LT(result.final_cost, result.initial_cost);
}

// =============================================================================
// Test 8: Pose-graph with PCG — rim (previously failed with Cholesky)
// =============================================================================
TEST(PCGSolverTest, Rim_PCG) {
  loader::G2OLoader loader("/data/se/g2o_datasets/rim.g2o");
  PoseGraphProblem problem;
  ASSERT_TRUE(loader.Load(problem));

  std::cout << "rim: " << problem.get_num_poses() << " poses, "
            << problem.get_num_edges() << " edges" << std::endl;

  LMConfig config;
  config.max_iterations = 50;
  config.verbose = true;
  config.initial_lambda = 1.0;
  config.linear_solver = LinearSolverType::PCG;
  config.pcg_max_iterations = 1000;
  config.pcg_tolerance = 1e-6;

  LMResult result = solvePoseGraph_GPU(problem, config);

  std::cout << "rim PCG result: " << result.termination_reason << std::endl;
  std::cout << "  Initial cost: " << result.initial_cost << std::endl;
  std::cout << "  Final cost:   " << result.final_cost << std::endl;
  std::cout << "  Cost ratio:   " << result.final_cost / result.initial_cost
            << std::endl;

  EXPECT_LT(result.final_cost, result.initial_cost);
}

// =============================================================================
// Test 9: Fixed vertex not optimized (PCG path)
// =============================================================================
TEST(PCGSolverTest, FixedVertexNotOptimized_PCG) {
  PoseGraphProblem problem;

  auto p0 = std::make_shared<Pose>(0, problem.memory_map);
  auto p1 = std::make_shared<Pose>(1, problem.memory_map);
  auto p2 = std::make_shared<Pose>(2, problem.memory_map);

  double* d0 = p0->mutable_data();
  d0[0] = 0;
  d0[1] = 0;
  d0[2] = 0;
  d0[3] = 0;
  d0[4] = 0;
  d0[5] = 0;
  d0[6] = 1;

  double* d1 = p1->mutable_data();
  d1[0] = 1;
  d1[1] = 0;
  d1[2] = 0;
  d1[3] = 0;
  d1[4] = 0;
  d1[5] = 0;
  d1[6] = 1;

  double* d2 = p2->mutable_data();
  d2[0] = 2;
  d2[1] = 0;
  d2[2] = 0;
  d2[3] = 0;
  d2[4] = 0;
  d2[5] = 0;
  d2[6] = 1;

  problem.poses = {p0, p1, p2};

  auto e01 = std::make_shared<PoseEdge>(0, 0, 1, problem.memory_map);
  double* m01 = e01->mutable_measurement_data();
  m01[0] = 1;
  m01[1] = 0;
  m01[2] = 0;
  m01[3] = 0;
  m01[4] = 0;
  m01[5] = 0;
  m01[6] = 1;

  auto e12 = std::make_shared<PoseEdge>(1, 1, 2, problem.memory_map);
  double* m12 = e12->mutable_measurement_data();
  m12[0] = 1;
  m12[1] = 0;
  m12[2] = 0;
  m12[3] = 0;
  m12[4] = 0;
  m12[5] = 0;
  m12[6] = 1;

  problem.edges = {e01, e12};
  problem.fixed_vertex_id = 0;

  double fixed_x = p0->x();
  double fixed_y = p0->y();
  double fixed_z = p0->z();

  LMConfig config;
  config.max_iterations = 10;
  config.verbose = true;
  config.linear_solver = LinearSolverType::PCG;

  LMResult result = solvePoseGraph_GPU(problem, config);

  EXPECT_NEAR(p0->x(), fixed_x, 1e-10);
  EXPECT_NEAR(p0->y(), fixed_y, 1e-10);
  EXPECT_NEAR(p0->z(), fixed_z, 1e-10);
  EXPECT_NEAR(result.final_cost, 0.0, 1e-6);
}

}  // namespace
}  // namespace solver
}  // namespace backend
}  // namespace substral
