#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "subastral/frontend/depth_initializer.hpp"
#include "subastral/frontend/feature_extractor.hpp"
#include "subastral/frontend/imu_initializer.hpp"
#include "subastral/frontend/imu_preintegrator.hpp"
#include "subastral/frontend/feature_matcher.hpp"
#include "subastral/frontend/two_view_geometry.hpp"
#include "subastral/loader/dataset_types.h"
#include "subastral/pipeline.hpp"
#include "subastral/types/map.h"
#include "subastral/types/sensor_config.h"
#include "subastral/types/transform_tree.h"
#include "viz/rerun/vfe_visualizer.hpp"

namespace substral {

/// Visual Front-End pipeline — monocular visual odometry.
///
/// Loads a TUM RGB-D sequence, extracts ORB features (GPU or CPU),
/// initializes a 3D map from two frames via essential matrix decomposition,
/// then tracks all subsequent frames via PnP against existing map points,
/// triangulating new points along the way.
class VisualFrontendPipeline : public Pipeline {
 public:
  VisualFrontendPipeline();

  const char* name() const override { return "Visual Frontend"; }
  bool load(const std::string& sequence_dir) override;
  void run(const PipelineConfig& config) override;

 private:
  /// Depth-based initialization from a single RGB-D frame.
  bool initializeFromDepth(int frame_idx, rerun_viz::VFEVisualizer* viz);

  /// Monocular initialization with parallax-based frame pair selection.
  bool initializeMonocular(rerun_viz::VFEVisualizer* viz);

  /// Two-view initialization from a specific frame pair (used by initializeMonocular).
  bool initialize(int idx1, int idx2, rerun_viz::VFEVisualizer* viz);

  /// Track a single frame via PnP against existing map points.
  bool trackFrame(int frame_idx, rerun_viz::VFEVisualizer* viz);

  /// Find the depth image closest in timestamp to a given RGB frame.
  /// Returns empty string if no depth frame is close enough.
  std::string findAssociatedDepthPath(int rgb_frame_idx,
                                      double max_dt = 0.02) const;

  /// Find nearest GT pose for a timestamp (used for evaluation/viz only).
  static Eigen::Matrix4d findNearestGTPose(
      const std::vector<loader::GroundTruthPose>& gt, double timestamp);

  loader::VisionDataset dataset_;
  SensorConfig sensor_config_;
  std::shared_ptr<TransformTree> tf_tree_;
  Map map_;
  std::unique_ptr<frontend::FeatureExtractor> extractor_;
  std::unique_ptr<frontend::FeatureMatcher> matcher_;
  frontend::TwoViewGeometry two_view_;
  frontend::DepthInitializer depth_init_;

  // Tracking state
  int tracking_failures_ = 0;
};

}  // namespace substral
