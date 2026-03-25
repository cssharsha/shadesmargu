#pragma once

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafeatures2d.hpp>

#include "subastral/frontend/feature_matcher.hpp"

namespace substral {
namespace frontend {

/// GPU brute-force matcher with Hamming distance for binary descriptors.
///
/// Descriptors are uploaded to GPU, matching runs on GPU,
/// results are downloaded back to CPU.
class FeatureMatcherGPU : public FeatureMatcher {
 public:
  FeatureMatcherGPU();

  std::vector<cv::DMatch> match(const cv::Mat& desc1, const cv::Mat& desc2,
                                float ratio_threshold = 0.75f) override;
  const char* name() const override { return "BF-GPU"; }

 private:
  cv::Ptr<cv::cuda::DescriptorMatcher> matcher_;
  cv::cuda::GpuMat gpu_desc1_;
  cv::cuda::GpuMat gpu_desc2_;
};

}  // namespace frontend
}  // namespace substral
