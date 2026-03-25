#include "subastral/frontend/feature_extractor_gpu.cuh"

namespace substral {
namespace frontend {

FeatureExtractorGPU::FeatureExtractorGPU(int n_features, float scale_factor,
                                         int n_levels, int fast_threshold) {
  orb_ = cv::cuda::ORB::create(n_features, scale_factor, n_levels,
                                /*edgeThreshold=*/31, /*firstLevel=*/0,
                                /*WTA_K=*/2, cv::ORB::HARRIS_SCORE,
                                /*patchSize=*/31, fast_threshold,
                                /*blurForDescriptor=*/false);
}

ExtractionResult FeatureExtractorGPU::extract(const cv::Mat& gray_image) {
  // Upload to GPU, then delegate to the GpuMat path
  gpu_image_.upload(gray_image);
  return extractGpu(gpu_image_);
}

ExtractionResult FeatureExtractorGPU::extractGpu(
    const cv::cuda::GpuMat& gpu_gray) {
  ExtractionResult result;

  // Detect + compute on GPU (async API uses GpuMat for keypoints)
  orb_->detectAndComputeAsync(gpu_gray, cv::noArray(), gpu_keypoints_,
                              gpu_descriptors_);

  // Download keypoints (convert from GPU compact format to std::vector)
  orb_->convert(gpu_keypoints_, result.keypoints);

  // Download descriptors
  gpu_descriptors_.download(result.descriptors);

  return result;
}

}  // namespace frontend
}  // namespace substral
