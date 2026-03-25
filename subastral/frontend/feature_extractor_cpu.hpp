#pragma once

#include <opencv2/features2d.hpp>

#include "subastral/frontend/feature_extractor.hpp"

namespace substral {
namespace frontend {

/// CPU ORB feature extractor using cv::ORB.
class FeatureExtractorCPU : public FeatureExtractor {
 public:
  /// Create with ORB parameters matching ORB-SLAM2 defaults.
  explicit FeatureExtractorCPU(int n_features = 1000, float scale_factor = 1.2f,
                               int n_levels = 8, int fast_threshold = 20);

  ExtractionResult extract(const cv::Mat& gray_image) override;
  const char* name() const override { return "ORB-CPU"; }

 private:
  cv::Ptr<cv::ORB> orb_;
};

}  // namespace frontend
}  // namespace substral
