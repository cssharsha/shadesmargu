#pragma once

#include <opencv2/features2d.hpp>

#include "subastral/frontend/feature_matcher.hpp"

namespace substral {
namespace frontend {

/// CPU brute-force matcher with Hamming distance for binary descriptors.
class FeatureMatcherCPU : public FeatureMatcher {
 public:
  FeatureMatcherCPU();

  std::vector<cv::DMatch> match(const cv::Mat& desc1, const cv::Mat& desc2,
                                float ratio_threshold = 0.75f) override;
  const char* name() const override { return "BF-CPU"; }

 private:
  cv::Ptr<cv::BFMatcher> matcher_;
};

}  // namespace frontend
}  // namespace substral
