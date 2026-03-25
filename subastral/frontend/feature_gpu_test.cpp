#include <algorithm>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

#include "gtest/gtest.h"
#include "subastral/frontend/feature_extractor_cpu.hpp"
#include "subastral/frontend/feature_extractor_gpu.cuh"
#include "subastral/frontend/feature_matcher_cpu.hpp"
#include "subastral/frontend/feature_matcher_gpu.cuh"

// Host-side test file — GPU calls go through OpenCV CUDA API (no <<<>>> syntax)

namespace substral {
namespace frontend {
namespace {

const std::string kImagePath =
    "/data/se/tum_datasets/rgbd_dataset_freiburg1_xyz/rgb/"
    "1305031102.175304.png";
const std::string kImagePath2 =
    "/data/se/tum_datasets/rgbd_dataset_freiburg1_xyz/rgb/"
    "1305031102.211214.png";

TEST(FeatureExtractorGPUTest, ExtractFromRealImage) {
  cv::Mat image = cv::imread(kImagePath, cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(image.empty());

  FeatureExtractorGPU extractor(1000);
  auto result = extractor.extract(image);

  // Should detect features
  EXPECT_GT(result.keypoints.size(), 100u);
  EXPECT_LE(result.keypoints.size(), 1000u);

  // Descriptors should match
  EXPECT_EQ(result.descriptors.rows, static_cast<int>(result.keypoints.size()));
  EXPECT_EQ(result.descriptors.cols, 32);
  EXPECT_EQ(result.descriptors.type(), CV_8UC1);

  std::cout << "GPU ORB: " << result.keypoints.size() << " features"
            << std::endl;
}

TEST(FeatureExtractorGPUTest, GPUvsCPUFeatureCount) {
  cv::Mat image = cv::imread(kImagePath, cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(image.empty());

  FeatureExtractorCPU cpu_ext(1000);
  FeatureExtractorGPU gpu_ext(1000);

  auto cpu_result = cpu_ext.extract(image);
  auto gpu_result = gpu_ext.extract(image);

  // GPU and CPU may not produce identical feature counts (different
  // implementations) but should be in the same ballpark
  int cpu_n = static_cast<int>(cpu_result.keypoints.size());
  int gpu_n = static_cast<int>(gpu_result.keypoints.size());

  std::cout << "CPU: " << cpu_n << " features, GPU: " << gpu_n << " features"
            << std::endl;

  // Both should detect a reasonable number
  EXPECT_GT(cpu_n, 100);
  EXPECT_GT(gpu_n, 100);

  // They should be within 50% of each other
  double ratio =
      static_cast<double>(std::min(cpu_n, gpu_n)) / std::max(cpu_n, gpu_n);
  EXPECT_GT(ratio, 0.5) << "CPU and GPU feature counts differ by more than 2x";
}

TEST(FeatureMatcherGPUTest, MatchConsecutiveFrames) {
  cv::Mat img1 = cv::imread(kImagePath, cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(kImagePath2, cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(img1.empty());
  ASSERT_FALSE(img2.empty());

  // Use CPU extractor (known good) to get descriptors
  FeatureExtractorCPU extractor(1000);
  auto r1 = extractor.extract(img1);
  auto r2 = extractor.extract(img2);

  FeatureMatcherGPU gpu_matcher;
  auto gpu_matches = gpu_matcher.match(r1.descriptors, r2.descriptors, 0.75f);

  // Should get a reasonable number of matches
  EXPECT_GT(gpu_matches.size(), 50u);

  // All indices valid
  for (const auto& m : gpu_matches) {
    EXPECT_GE(m.queryIdx, 0);
    EXPECT_LT(m.queryIdx, static_cast<int>(r1.keypoints.size()));
    EXPECT_GE(m.trainIdx, 0);
    EXPECT_LT(m.trainIdx, static_cast<int>(r2.keypoints.size()));
  }

  std::cout << "GPU BF match: " << gpu_matches.size() << " good matches"
            << std::endl;
}

TEST(FeatureMatcherGPUTest, GPUvsCPUMatchCount) {
  cv::Mat img1 = cv::imread(kImagePath, cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(kImagePath2, cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(img1.empty());
  ASSERT_FALSE(img2.empty());

  FeatureExtractorCPU extractor(1000);
  auto r1 = extractor.extract(img1);
  auto r2 = extractor.extract(img2);

  FeatureMatcherCPU cpu_matcher;
  FeatureMatcherGPU gpu_matcher;

  auto cpu_matches = cpu_matcher.match(r1.descriptors, r2.descriptors, 0.75f);
  auto gpu_matches = gpu_matcher.match(r1.descriptors, r2.descriptors, 0.75f);

  std::cout << "CPU matches: " << cpu_matches.size()
            << ", GPU matches: " << gpu_matches.size() << std::endl;

  // GPU and CPU BF matching with same descriptors should produce
  // identical or very similar results
  // (Both use exact Hamming distance, so results should match exactly)
  EXPECT_EQ(cpu_matches.size(), gpu_matches.size())
      << "CPU and GPU match counts should be identical for BF+Hamming";

  // Verify same matches (sort by queryIdx for comparison)
  if (cpu_matches.size() == gpu_matches.size()) {
    auto sort_by_query = [](const cv::DMatch& a, const cv::DMatch& b) {
      return a.queryIdx < b.queryIdx ||
             (a.queryIdx == b.queryIdx && a.trainIdx < b.trainIdx);
    };
    std::sort(cpu_matches.begin(), cpu_matches.end(), sort_by_query);
    std::sort(gpu_matches.begin(), gpu_matches.end(), sort_by_query);

    for (size_t i = 0; i < cpu_matches.size(); ++i) {
      EXPECT_EQ(cpu_matches[i].queryIdx, gpu_matches[i].queryIdx);
      EXPECT_EQ(cpu_matches[i].trainIdx, gpu_matches[i].trainIdx);
      EXPECT_FLOAT_EQ(cpu_matches[i].distance, gpu_matches[i].distance);
    }
  }
}

}  // namespace
}  // namespace frontend
}  // namespace substral
