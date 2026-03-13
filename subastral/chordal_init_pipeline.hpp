#pragma once

#include "subastral/backend/common.h"
#include "subastral/pipeline.hpp"

namespace substral {

/// Chordal-relaxation initialized Pose-Graph SLAM pipeline.
///
/// Uses a two-stage initialization before running the full nonlinear solver:
///   1. Rotation estimation via chordal relaxation (sparse linear solve + SVD
///      projection to SO(3))
///   2. Translation recovery with fixed rotations (sparse linear solve)
///   3. Full pose-graph optimization from the initialized poses
///
/// This produces globally-consistent initial poses that avoid the local minima
/// that trap the standard odometry-initialized solver on datasets like cubicle
/// and rim.
///
/// Reference: Carlone et al., "Initialization Techniques for 3D SLAM: a Survey
/// on Rotation Estimation and its Use in Pose Graph Optimization", ICRA 2015.
class ChordalInitPipeline : public Pipeline {
 public:
  const char* name() const override { return "Pose-Graph SLAM (chordal init)"; }
  bool load(const std::string& filename) override;
  void run(const PipelineConfig& config) override;

 private:
  backend::PoseGraphProblem problem_;

  /// Step 1: Solve for rotations via chordal relaxation.
  /// Minimizes sum_edges ||R_i - R_ij * R_j||_F^2 as a linear system in the
  /// 9 entries of each R_i, then projects to SO(3) via SVD.
  void initializeRotations();

  /// Step 2: Recover translations with rotations fixed.
  /// Minimizes sum_edges ||R_i^T * (t_j - t_i) - t_ij||^2 as a sparse
  /// linear system in the 3 entries of each t_i.
  void initializeTranslations();
};

}  // namespace substral
