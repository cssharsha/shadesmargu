#include <iostream>
#include <opencv2/imgcodecs.hpp>

#include "gtest/gtest.h"
#include "subastral/frontend/feature_extractor_cpu.hpp"
#include "subastral/frontend/feature_matcher_cpu.hpp"

namespace substral {
namespace frontend {
namespace {

// Use the first image from the downloaded fr1/xyz dataset
const std::string kImagePath =
    "/data/se/tum_datasets/rgbd_dataset_freiburg1_xyz/rgb/"
    "1305031102.175304.png";

// Second image (consecutive frame) for matching test
const std::string kImagePath2 =
    "/data/se/tum_datasets/rgbd_dataset_freiburg1_xyz/rgb/"
    "1305031102.211214.png";

TEST(FeatureExtractorCPUTest, ExtractFromRealImage) {
  cv::Mat image = cv::imread(kImagePath, cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(image.empty()) << "Failed to load image: " << kImagePath;
  ASSERT_EQ(image.channels(), 1);

  FeatureExtractorCPU extractor(1000);
  auto result = extractor.extract(image);

  // Should detect a reasonable number of features
  EXPECT_GT(result.keypoints.size(), 100u);
  EXPECT_LE(result.keypoints.size(), 1000u);

  // Descriptors should match keypoint count
  EXPECT_EQ(result.descriptors.rows, static_cast<int>(result.keypoints.size()));
  EXPECT_EQ(result.descriptors.cols, 32);  // ORB = 256 bits = 32 bytes
  EXPECT_EQ(result.descriptors.type(), CV_8UC1);

  std::cout << "CPU ORB: " << result.keypoints.size() << " features from "
            << image.cols << "x" << image.rows << " image" << std::endl;
}

TEST(FeatureExtractorCPUTest, EmptyImage) {
  cv::Mat empty;
  FeatureExtractorCPU extractor;
  auto result = extractor.extract(empty);
  EXPECT_TRUE(result.keypoints.empty());
  EXPECT_TRUE(result.descriptors.empty());
}

TEST(FeatureMatcherCPUTest, MatchConsecutiveFrames) {
  cv::Mat img1 = cv::imread(kImagePath, cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(kImagePath2, cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(img1.empty());
  ASSERT_FALSE(img2.empty());

  FeatureExtractorCPU extractor(1000);
  auto r1 = extractor.extract(img1);
  auto r2 = extractor.extract(img2);

  ASSERT_GT(r1.keypoints.size(), 0u);
  ASSERT_GT(r2.keypoints.size(), 0u);

  FeatureMatcherCPU matcher;
  auto matches = matcher.match(r1.descriptors, r2.descriptors, 0.75f);

  // Consecutive frames should have many matches
  EXPECT_GT(matches.size(), 50u);

  // All match indices should be valid
  for (const auto& m : matches) {
    EXPECT_GE(m.queryIdx, 0);
    EXPECT_LT(m.queryIdx, static_cast<int>(r1.keypoints.size()));
    EXPECT_GE(m.trainIdx, 0);
    EXPECT_LT(m.trainIdx, static_cast<int>(r2.keypoints.size()));
    EXPECT_GE(m.distance, 0.0f);
  }

  std::cout << "CPU BF match: " << matches.size() << " good matches from "
            << r1.keypoints.size() << " and " << r2.keypoints.size()
            << " features" << std::endl;
}

TEST(FeatureMatcherCPUTest, EmptyDescriptors) {
  FeatureMatcherCPU matcher;
  cv::Mat empty;
  cv::Mat desc = cv::Mat::ones(10, 32, CV_8UC1);

  auto m1 = matcher.match(empty, desc);
  EXPECT_TRUE(m1.empty());

  auto m2 = matcher.match(desc, empty);
  EXPECT_TRUE(m2.empty());
}

}  // namespace
}  // namespace frontend
}  // namespace substral
