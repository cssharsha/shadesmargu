#include "subastral/frontend/image_preprocessor_gpu.cuh"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace substral {
namespace frontend {

void ImagePreprocessorGPU::init(const CameraIntrinsics& intrinsics,
                                cv::Size image_size) {
  has_distortion_ = !intrinsics.isUndistorted();

  if (has_distortion_) {
    // Compute undistortion maps on CPU, then upload to GPU once
    cv::Mat K = intrinsics.cameraMatrix();
    cv::Mat dist = intrinsics.distCoeffs();

    cv::Mat map1_cpu, map2_cpu;
    cv::initUndistortRectifyMap(K, dist, cv::Mat(), K, image_size, CV_32FC1,
                                map1_cpu, map2_cpu);

    gpu_map1_.upload(map1_cpu);
    gpu_map2_.upload(map2_cpu);
  }

  initialized_ = true;
}

const cv::cuda::GpuMat& ImagePreprocessorGPU::preprocess(
    const cv::Mat& image) {
  // Upload to GPU
  gpu_uploaded_.upload(image);

  // Convert to grayscale if needed
  if (image.channels() == 3) {
    cv::cuda::cvtColor(gpu_uploaded_, gpu_gray_, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    cv::cuda::cvtColor(gpu_uploaded_, gpu_gray_, cv::COLOR_BGRA2GRAY);
  } else {
    gpu_gray_ = gpu_uploaded_;  // Already grayscale
  }

  // Undistort if camera has distortion
  if (has_distortion_ && initialized_) {
    cv::cuda::remap(gpu_gray_, gpu_undistorted_, gpu_map1_, gpu_map2_,
                    cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  } else {
    gpu_undistorted_ = gpu_gray_;
  }

  return gpu_undistorted_;
}

}  // namespace frontend
}  // namespace substral
