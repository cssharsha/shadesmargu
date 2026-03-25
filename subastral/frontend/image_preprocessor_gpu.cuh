#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

#include "subastral/types/camera.h"

namespace substral {
namespace frontend {

/// GPU image preprocessor: upload, grayscale conversion, and undistortion.
///
/// Keeps the image on GPU throughout the pipeline so that the GPU feature
/// extractor can operate directly on the device-resident result without
/// a redundant host→device transfer.
///
/// Typical flow:
///   1. preprocess(bgr_image)  — upload + cvtColor + remap
///   2. gpu_extractor.extractGpu(preprocessor.result())
///
/// The undistortion maps are precomputed once in init() and reused for
/// every frame (all frames in a TUM sequence share the same intrinsics).
class ImagePreprocessorGPU {
 public:
  /// Initialize with camera intrinsics and image size.
  /// Precomputes the undistortion remap tables on GPU.
  void init(const CameraIntrinsics& intrinsics, cv::Size image_size);

  /// Preprocess a BGR (or grayscale) image:
  ///   - Upload to GPU
  ///   - Convert to grayscale if needed
  ///   - Undistort via remap
  /// Returns a reference to the device-resident result.
  const cv::cuda::GpuMat& preprocess(const cv::Mat& image);

  /// Access the preprocessed result on GPU (valid after preprocess()).
  const cv::cuda::GpuMat& result() const { return gpu_undistorted_; }

  /// Whether init() has been called.
  bool initialized() const { return initialized_; }

 private:
  cv::cuda::GpuMat gpu_map1_, gpu_map2_;  // Precomputed undistortion maps
  cv::cuda::GpuMat gpu_uploaded_;         // Raw uploaded image
  cv::cuda::GpuMat gpu_gray_;            // After grayscale conversion
  cv::cuda::GpuMat gpu_undistorted_;     // After undistortion (final result)
  bool initialized_ = false;
  bool has_distortion_ = true;
};

}  // namespace frontend
}  // namespace substral
