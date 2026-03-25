#include "subastral/frontend/image_preprocessor_gpu.cuh"

#include <chrono>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

#include "gtest/gtest.h"
#include "subastral/frontend/feature_extractor_cpu.hpp"
#include "subastral/frontend/feature_extractor_gpu.cuh"
#include "subastral/loader/tum_loader.h"

// Tests the full GPU pipeline: preprocessor → GPU extractor (no re-upload)

namespace substral {
namespace frontend {
namespace {

const std::string kDatasetPath =
    "/data/se/tum_datasets/rgbd_dataset_freiburg1_xyz";
const std::string kImagePath =
    "/data/se/tum_datasets/rgbd_dataset_freiburg1_xyz/rgb/"
    "1305031102.175304.png";

TEST(ImagePreprocessorGPUTest, PreprocessAndExtract) {
  // Load intrinsics
  loader::TUMLoader loader;
  ASSERT_TRUE(loader.load(kDatasetPath));
  const auto& intrinsics = loader.dataset().intrinsics;

  // Load a color image (BGR)
  cv::Mat bgr_image = cv::imread(kImagePath, cv::IMREAD_COLOR);
  ASSERT_FALSE(bgr_image.empty());
  ASSERT_EQ(bgr_image.channels(), 3);

  // Init preprocessor
  ImagePreprocessorGPU preprocessor;
  preprocessor.init(intrinsics, bgr_image.size());
  ASSERT_TRUE(preprocessor.initialized());

  // Preprocess: upload + grayscale + undistort on GPU
  const auto& gpu_gray = preprocessor.preprocess(bgr_image);

  // Verify result is on GPU, single-channel, same size
  EXPECT_EQ(gpu_gray.channels(), 1);
  EXPECT_EQ(gpu_gray.rows, bgr_image.rows);
  EXPECT_EQ(gpu_gray.cols, bgr_image.cols);

  // Extract features directly from GPU-resident image (no re-upload)
  FeatureExtractorGPU extractor(1000);
  auto result = extractor.extractGpu(gpu_gray);

  EXPECT_GT(result.keypoints.size(), 100u);
  EXPECT_EQ(result.descriptors.rows, static_cast<int>(result.keypoints.size()));

  std::cout << "Preprocessor + GPU ORB: " << result.keypoints.size()
            << " features from undistorted image" << std::endl;
}

TEST(ImagePreprocessorGPUTest, TimingComparison) {
  loader::TUMLoader loader;
  ASSERT_TRUE(loader.load(kDatasetPath));
  const auto& intrinsics = loader.dataset().intrinsics;

  cv::Mat bgr_image = cv::imread(kImagePath, cv::IMREAD_COLOR);
  ASSERT_FALSE(bgr_image.empty());

  ImagePreprocessorGPU preprocessor;
  preprocessor.init(intrinsics, bgr_image.size());

  FeatureExtractorGPU gpu_extractor(1000);

  // Warm up
  preprocessor.preprocess(bgr_image);
  gpu_extractor.extractGpu(preprocessor.result());

  // Benchmark: full GPU pipeline (preprocess + extract, no re-upload)
  constexpr int kIters = 20;
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIters; ++i) {
    preprocessor.preprocess(bgr_image);
    gpu_extractor.extractGpu(preprocessor.result());
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  double gpu_pipeline_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count() / kIters;

  // Benchmark: naive path (imread grayscale + GPU extract with upload)
  cv::Mat gray;
  cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
  t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIters; ++i) {
    gpu_extractor.extract(gray);
  }
  t1 = std::chrono::high_resolution_clock::now();
  double naive_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count() / kIters;

  // Benchmark: CPU extraction for reference
  FeatureExtractorCPU cpu_extractor(1000);
  t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIters; ++i) {
    cpu_extractor.extract(gray);
  }
  t1 = std::chrono::high_resolution_clock::now();
  double cpu_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count() / kIters;

  std::cout << "Timing (avg over " << kIters << " iterations):\n"
            << "  CPU ORB:                     " << std::fixed
            << std::setprecision(2) << cpu_ms << " ms\n"
            << "  GPU ORB (naive, re-upload):   " << naive_ms << " ms\n"
            << "  GPU pipeline (preproc+ORB):   " << gpu_pipeline_ms << " ms\n"
            << "  Speedup (CPU vs GPU pipeline): " << std::setprecision(1)
            << cpu_ms / gpu_pipeline_ms << "x\n";
}

TEST(ImagePreprocessorGPUTest, GrayscaleInput) {
  loader::TUMLoader loader;
  ASSERT_TRUE(loader.load(kDatasetPath));

  // Load as grayscale directly
  cv::Mat gray = cv::imread(kImagePath, cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(gray.empty());
  ASSERT_EQ(gray.channels(), 1);

  ImagePreprocessorGPU preprocessor;
  preprocessor.init(loader.dataset().intrinsics, gray.size());

  const auto& gpu_result = preprocessor.preprocess(gray);
  EXPECT_EQ(gpu_result.channels(), 1);
  EXPECT_EQ(gpu_result.rows, gray.rows);
}

TEST(ImagePreprocessorGPUTest, UndistortedCamera) {
  // Freiburg3 has no distortion — preprocessor should skip remap
  CameraIntrinsics fr3;
  fr3.fx = 535.4;
  fr3.fy = 539.2;
  fr3.cx = 320.1;
  fr3.cy = 247.6;
  // All distortion coefficients default to 0.0

  EXPECT_TRUE(fr3.isUndistorted());

  cv::Mat gray = cv::imread(kImagePath, cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(gray.empty());

  ImagePreprocessorGPU preprocessor;
  preprocessor.init(fr3, gray.size());

  const auto& gpu_result = preprocessor.preprocess(gray);
  EXPECT_EQ(gpu_result.channels(), 1);
  EXPECT_EQ(gpu_result.rows, gray.rows);
}

}  // namespace
}  // namespace frontend
}  // namespace substral
