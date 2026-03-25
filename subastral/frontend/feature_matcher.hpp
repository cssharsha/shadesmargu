#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace substral {
namespace frontend {

/// Abstract interface for feature matching.
///
/// Implementations may use CPU or GPU brute-force matching with
/// Hamming distance (for binary descriptors like ORB).
class FeatureMatcher {
 public:
  virtual ~FeatureMatcher() = default;

  /// Match descriptors from two images using Lowe's ratio test.
  /// Returns only good matches that pass the ratio threshold.
  ///
  /// @param desc1 Descriptors from image 1 (NxD, CV_8U for ORB)
  /// @param desc2 Descriptors from image 2 (MxD, CV_8U for ORB)
  /// @param ratio_threshold Lowe's ratio test threshold (default 0.75)
  /// @return Vector of good matches (queryIdx → desc1, trainIdx → desc2)
  virtual std::vector<cv::DMatch> match(const cv::Mat& desc1,
                                        const cv::Mat& desc2,
                                        float ratio_threshold = 0.75f) = 0;

  /// Human-readable name (e.g., "BF-CPU", "BF-GPU").
  virtual const char* name() const = 0;
};

}  // namespace frontend
}  // namespace substral
