#include "subastral/subastral.hpp"

#include <iostream>
#include <stdexcept>

#include "subastral/ba_pipeline.hpp"
#include "subastral/pose_graph_pipeline.hpp"

namespace substral {

Subastral::Subastral() {
  // Default to BA pipeline
  pipeline_ = std::make_unique<BAPipeline>();
}

void Subastral::createPipeline(const std::string& mode) {
  if (mode == "pose-graph") {
    pipeline_ = std::make_unique<PoseGraphPipeline>();
  } else if (mode == "ba") {
    pipeline_ = std::make_unique<BAPipeline>();
  } else {
    throw std::invalid_argument("Unknown pipeline mode: " + mode +
                                " (expected \"ba\" or \"pose-graph\")");
  }
}

bool Subastral::load(const std::string& filename) {
  if (!pipeline_) {
    std::cerr << "No pipeline created. Call createPipeline() first."
              << std::endl;
    return false;
  }
  return pipeline_->load(filename);
}

void Subastral::run() {
  if (!pipeline_) {
    std::cerr << "No pipeline created. Call createPipeline() first."
              << std::endl;
    return;
  }
  std::cout << "[pipeline: " << pipeline_->name() << "]" << std::endl;
  pipeline_->run(config_);
}

}  // namespace substral
