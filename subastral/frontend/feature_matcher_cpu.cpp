#include "subastral/frontend/feature_matcher_cpu.hpp"

namespace substral {
namespace frontend {

FeatureMatcherCPU::FeatureMatcherCPU() {
  matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, /*crossCheck=*/false);
}

std::vector<cv::DMatch> FeatureMatcherCPU::match(const cv::Mat& desc1,
                                                 const cv::Mat& desc2,
                                                 float ratio_threshold) {
  std::vector<cv::DMatch> good_matches;

  if (desc1.empty() || desc2.empty()) {
    return good_matches;
  }

  // kNN match with k=2 for Lowe's ratio test
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher_->knnMatch(desc1, desc2, knn_matches, 2);

  // Apply ratio test
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
