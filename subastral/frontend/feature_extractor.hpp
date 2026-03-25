#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace substral {
namespace frontend {

/// Result of feature extraction from a single image.
struct ExtractionResult {
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;  // On CPU (downloaded from GPU if needed)
};

/// Abstract interface for feature extraction.
///
/// Implementations may use CPU (cv::ORB) or GPU (cv::cuda::ORB).
/// The interface is intentionally simple: grayscale image in, features out.
class FeatureExtractor {
 public:
  virtual ~FeatureExtractor() = default;

  /// Extract features from a grayscale image.
  /// The image should be single-channel (CV_8UC1).
  virtual ExtractionResult extract(const cv::Mat& gray_image) = 0;

  /// Human-readable name (e.g., "ORB-CPU", "ORB-GPU").
  virtual const char* name() const = 0;
};

}  // namespace frontend
}  // namespace substral
