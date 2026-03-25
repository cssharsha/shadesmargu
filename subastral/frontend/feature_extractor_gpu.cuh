#pragma once

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaimgproc.hpp>

#include "subastral/frontend/feature_extractor.hpp"

namespace substral {
namespace frontend {

/// GPU ORB feature extractor using cv::cuda::ORB.
///
/// The image is uploaded to GPU, converted to grayscale if needed,
/// and ORB detection + descriptor computation runs entirely on GPU.
/// Results are downloaded back to CPU.
class FeatureExtractorGPU : public FeatureExtractor {
 public:
  /// Create with ORB parameters matching ORB-SLAM2 defaults.
  explicit FeatureExtractorGPU(int n_features = 1000,
                               float scale_factor = 1.2f, int n_levels = 8,
                               int fast_threshold = 20);

  ExtractionResult extract(const cv::Mat& gray_image) override;

  /// Extract features from a GpuMat already on device (avoids re-upload).
  /// The image must be single-channel CV_8UC1.
  ExtractionResult extractGpu(const cv::cuda::GpuMat& gpu_gray);

  const char* name() const override { return "ORB-GPU"; }

 private:
  cv::Ptr<cv::cuda::ORB> orb_;
  cv::cuda::GpuMat gpu_image_;
  cv::cuda::GpuMat gpu_keypoints_;
  cv::cuda::GpuMat gpu_descriptors_;
};

}  // namespace frontend
}  // namespace substral
