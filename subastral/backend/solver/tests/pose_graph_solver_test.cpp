#include "subastral/backend/solver/pose_graph_solver.cuh"

#include <cmath>
#include <iostream>

#include "gtest/gtest.h"
#include "subastral/backend/common.h"
#include "subastral/loader/g2o_loader.h"

namespace substral {
namespace backend {
namespace solver {
namespace {

// Host wrapper declarations (defined in .cu files, callable from .cpp)
// These are declared in the .cuh header.

TEST(PoseGraphSolverTest, Torus3D) {
  // Load dataset
  loader::G2OLoader loader("/data/se/g2o_datasets/torus3D.g2o");
  PoseGraphProblem problem;
  ASSERT_TRUE(loader.Load(problem));

  std::cout << "Loaded: " << problem.get_num_poses() << " poses, "
            << problem.get_num_edges() << " edges" << std::endl;

  // Configure solver
  LMConfig config;
  config.max_iterations = 30;
  config.verbose = true;
  config.initial_lambda = 1e-3;
  config.cost_tolerance = 1e-4;

  // Solve
  LMResult result = solvePoseGraph_GPU(problem, config);

  std::cout << "Result: " << result.termination_reason << std::endl;
  std::cout << "  Initial cost: " << result.initial_cost << std::endl;
  std::cout << "  Final cost:   " << result.final_cost << std::endl;
  std::cout << "  Iterations:   " << result.iterations << std::endl;
  std::cout << "  Cost ratio:   " << result.final_cost / result.initial_cost
            << std::endl;

  // The solver should reduce cost significantly
  EXPECT_LT(result.final_cost, result.initial_cost);
  // torus3D should converge well — expect > 10x cost reduction
  EXPECT_LT(result.final_cost, result.initial_cost * 0.1);
}

TEST(PoseGraphSolverTest, ParkingGarage) {
  loader::G2OLoader loader("/data/se/g2o_datasets/parking-garage.g2o");
  PoseGraphProblem problem;
  ASSERT_TRUE(loader.Load(problem));

  LMConfig config;
  config.max_iterations = 30;
  config.verbose = true;
  config.initial_lambda = 1e-3;

  LMResult result = solvePoseGraph_GPU(problem, config);

  std::cout << "parking-garage result: " << result.termination_reason
            << std::endl;
  std::cout << "  Initial cost: " << result.initial_cost << std::endl;
  std::cout << "  Final cost:   " << result.final_cost << std::endl;
  std::cout << "  Cost ratio:   " << result.final_cost / result.initial_cost
            << std::endl;

  EXPECT_LT(result.final_cost, result.initial_cost);
}

TEST(PoseGraphSolverTest, FixedVertexNotOptimized) {
  // Create a tiny 3-pose problem and verify the fixed pose doesn't move
  PoseGraphProblem problem;

  // 3 poses in a line
  auto p0 = std::make_shared<Pose>(0, problem.memory_map);
  auto p1 = std::make_shared<Pose>(1, problem.memory_map);
  auto p2 = std::make_shared<Pose>(2, problem.memory_map);

  // Set positions
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

  // Edge 0→1: measurement = (1,0,0, identity quat)
  auto e01 = std::make_shared<PoseEdge>(0, 0, 1, problem.memory_map);
  double* m01 = e01->mutable_measurement_data();
  m01[0] = 1;
  m01[1] = 0;
  m01[2] = 0;
  m01[3] = 0;
  m01[4] = 0;
  m01[5] = 0;
  m01[6] = 1;

  // Edge 1→2: measurement = (1,0,0, identity quat)
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

  // Save fixed pose
  double fixed_x = p0->x();
  double fixed_y = p0->y();
  double fixed_z = p0->z();

  LMConfig config;
  config.max_iterations = 10;
  config.verbose = true;

  LMResult result = solvePoseGraph_GPU(problem, config);

  // Fixed pose should not have moved
  EXPECT_NEAR(p0->x(), fixed_x, 1e-10);
  EXPECT_NEAR(p0->y(), fixed_y, 1e-10);
  EXPECT_NEAR(p0->z(), fixed_z, 1e-10);

  // Cost should be near zero (poses already consistent with measurements)
  EXPECT_NEAR(result.final_cost, 0.0, 1e-6);
}

}  // namespace
}  // namespace solver
}  // namespace backend
}  // namespace substral
