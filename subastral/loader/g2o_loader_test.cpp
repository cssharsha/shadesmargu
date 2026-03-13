#include "subastral/loader/g2o_loader.h"

#include <cmath>
#include <iostream>

#include "gtest/gtest.h"

namespace substral {
namespace loader {
namespace {

// Test loading the torus3D.g2o dataset
TEST(G2OLoaderTest, LoadTorus3D) {
  const std::string path = "/data/se/g2o_datasets/torus3D.g2o";
  G2OLoader loader(path);
  backend::PoseGraphProblem problem;

  ASSERT_TRUE(loader.Load(problem));

  // torus3D has ~800 vertices and ~800+ edges
  EXPECT_GT(problem.get_num_poses(), 0);
  EXPECT_GT(problem.get_num_edges(), 0);

  std::cout << "torus3D: " << problem.get_num_poses() << " poses, "
            << problem.get_num_edges() << " edges, "
            << problem.get_num_loop_closures() << " loop closures" << std::endl;

  // Verify first pose has valid data (not all zeros)
  ASSERT_FALSE(problem.poses.empty());
  const double* first = problem.poses[0]->data();
  // qw should be close to 1 for identity-ish quaternion
  double qnorm = std::sqrt(first[3] * first[3] + first[4] * first[4] +
                           first[5] * first[5] + first[6] * first[6]);
  EXPECT_NEAR(qnorm, 1.0, 1e-6) << "Quaternion should be unit";
}

// Test loading the parking-garage dataset
TEST(G2OLoaderTest, LoadParkingGarage) {
  const std::string path = "/data/se/g2o_datasets/parking-garage.g2o";
  G2OLoader loader(path);
  backend::PoseGraphProblem problem;

  ASSERT_TRUE(loader.Load(problem));

  EXPECT_GT(problem.get_num_poses(), 0);
  EXPECT_GT(problem.get_num_edges(), 0);

  std::cout << "parking-garage: " << problem.get_num_poses() << " poses, "
            << problem.get_num_edges() << " edges, "
            << problem.get_num_loop_closures() << " loop closures" << std::endl;
}

// Test that the memory map is consistent
TEST(G2OLoaderTest, MemoryMapConsistency) {
  const std::string path = "/data/se/g2o_datasets/torus3D.g2o";
  G2OLoader loader(path);
  backend::PoseGraphProblem problem;

  ASSERT_TRUE(loader.Load(problem));

  int n_poses = problem.get_num_poses();
  int n_edges = problem.get_num_edges();

  // Check contiguous storage sizes
  EXPECT_EQ(static_cast<int>(problem.memory_map->poses.size()),
            n_poses * backend::POSE_STRIDE);
  EXPECT_EQ(static_cast<int>(problem.memory_map->pose_edges.size()),
            n_edges * backend::POSE_EDGE_STRIDE);
  EXPECT_EQ(static_cast<int>(problem.memory_map->pose_edge_from_indices.size()),
            n_edges);
  EXPECT_EQ(static_cast<int>(problem.memory_map->pose_edge_to_indices.size()),
            n_edges);

  // Each pose's data() should point into the contiguous array
  for (const auto& pose : problem.poses) {
    const double* d = pose->data();
    const double* base = problem.pose_data();
    ptrdiff_t offset = d - base;
    EXPECT_GE(offset, 0);
    EXPECT_LT(offset, n_poses * backend::POSE_STRIDE);
  }
}

// Test that info matrix permutation is applied (rotation-first ordering)
TEST(G2OLoaderTest, InfoMatrixPermutation) {
  const std::string path = "/data/se/g2o_datasets/torus3D.g2o";
  G2OLoader loader(path);
  backend::PoseGraphProblem problem;

  ASSERT_TRUE(loader.Load(problem));
  ASSERT_FALSE(problem.edges.empty());

  // The info matrix should be symmetric after permutation
  const double* info = problem.edges[0]->info_data();
  for (int r = 0; r < 6; ++r) {
    for (int c = r + 1; c < 6; ++c) {
      EXPECT_NEAR(info[r * 6 + c], info[c * 6 + r], 1e-12)
          << "Info matrix not symmetric at (" << r << "," << c << ")";
    }
  }

  // The info matrix should be positive (all diagonal entries > 0)
  for (int i = 0; i < 6; ++i) {
    EXPECT_GT(info[i * 6 + i], 0.0)
        << "Info matrix diagonal entry " << i << " should be positive";
  }
}

// Test PoseGraphProblem helper methods
TEST(G2OLoaderTest, ProblemHelpers) {
  const std::string path = "/data/se/g2o_datasets/torus3D.g2o";
  G2OLoader loader(path);
  backend::PoseGraphProblem problem;

  ASSERT_TRUE(loader.Load(problem));

  // Set fixed vertex to first pose
  problem.fixed_vertex_id = problem.poses[0]->get_id();

  auto id_map = problem.build_vertex_index_map();
  EXPECT_EQ(id_map[problem.fixed_vertex_id], -1);

  // All other vertices should have non-negative indices
  int max_idx = -1;
  for (const auto& kv : id_map) {
    if (kv.first != problem.fixed_vertex_id) {
      EXPECT_GE(kv.second, 0);
      max_idx = std::max(max_idx, kv.second);
    }
  }
  EXPECT_EQ(max_idx, problem.get_num_poses() - 2);

  // DOF check
  EXPECT_EQ(problem.get_num_free_params(), 6 * (problem.get_num_poses() - 1));

  // Sorted vertex IDs should be in ascending order
  auto ids = problem.sorted_vertex_ids();
  for (size_t i = 1; i < ids.size(); ++i) {
    EXPECT_GT(ids[i], ids[i - 1]);
  }
}

// Test loading a nonexistent file
TEST(G2OLoaderTest, NonexistentFile) {
  G2OLoader loader("/nonexistent/path.g2o");
  backend::PoseGraphProblem problem;
  EXPECT_FALSE(loader.Load(problem));
}

}  // namespace
}  // namespace loader
}  // namespace substral
