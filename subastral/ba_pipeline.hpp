#pragma once

#include "subastral/backend/common.h"
#include "subastral/pipeline.hpp"

namespace substral {

/// Bundle Adjustment pipeline — loads BAL datasets and runs LM optimization.
class BAPipeline : public Pipeline {
 public:
  const char* name() const override { return "Bundle Adjustment"; }
  bool load(const std::string& filename) override;
  void run(const PipelineConfig& config) override;

 private:
  backend::BAProblem problem_;

  void printProblemStats() const;
  void printErrorStats(const std::string& label, bool gpu);
};

}  // namespace substral
