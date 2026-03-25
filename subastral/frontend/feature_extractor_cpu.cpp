#include "subastral/frontend/feature_extractor_cpu.hpp"

namespace substral {
namespace frontend {

FeatureExtractorCPU::FeatureExtractorCPU(int n_features, float scale_factor,
                                         int n_levels, int fast_threshold) {
  orb_ = cv::ORB::create(n_features, scale_factor, n_levels,
                         /*edgeThreshold=*/31, /*firstLevel=*/0,
                         /*WTA_K=*/2, cv::ORB::HARRIS_SCORE,
                         /*patchSize=*/31, fast_threshold);
}

ExtractionResult FeatureExtractorCPU::extract(const cv::Mat& gray_image) {
  ExtractionResult result;
  orb_->detectAndCompute(gray_image, cv::noArray(), result.keypoints,
                         result.descriptors);
  return result;
}

}  // namespace frontend
}  // namespace substral
