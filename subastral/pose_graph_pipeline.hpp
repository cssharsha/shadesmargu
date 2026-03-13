#pragma once

#include "subastral/backend/common.h"
#include "subastral/pipeline.hpp"

namespace substral {

/// Pose-Graph SLAM pipeline — loads g2o datasets and runs GPU LM optimization.
class PoseGraphPipeline : public Pipeline {
 public:
  const char* name() const override { return "Pose-Graph SLAM"; }
  bool load(const std::string& filename) override;
  void run(const PipelineConfig& config) override;

 private:
  backend::PoseGraphProblem problem_;
};

}  // namespace substral
