#include "subastral/frontend/feature_matcher_gpu.cuh"

namespace substral {
namespace frontend {

FeatureMatcherGPU::FeatureMatcherGPU() {
  matcher_ = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
}

std::vector<cv::DMatch> FeatureMatcherGPU::match(const cv::Mat& desc1,
                                                  const cv::Mat& desc2,
                                                  float ratio_threshold) {
  std::vector<cv::DMatch> good_matches;

  if (desc1.empty() || desc2.empty()) {
    return good_matches;
  }

  // Upload descriptors to GPU
  gpu_desc1_.upload(desc1);
  gpu_desc2_.upload(desc2);

  // kNN match with k=2 on GPU
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher_->knnMatch(gpu_desc1_, gpu_desc2_, knn_matches, 2);

  // Apply Lowe's ratio test (on CPU — the match results are already downloaded)
  good_matches.reserve(knn_matches.size());
  for (const auto& m : knn_matches) {
    if (m.size() >= 2 && m[0].distance < ratio_threshold * m[1].distance) {
      good_matches.push_back(m[0]);
    }
  }

  return good_matches;
}

}  // namespace frontend
}  // namespace substral
