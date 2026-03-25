#include "subastral/frontend/two_view_geometry.hpp"

#include <cmath>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

#include "gtest/gtest.h"
#include "subastral/frontend/feature_extractor_cpu.hpp"
#include "subastral/frontend/feature_matcher_cpu.hpp"
#include "subastral/loader/tum_loader.h"

namespace substral {
namespace frontend {
namespace {

const std::string kDatasetPath =
    "/data/se/tum_datasets/rgbd_dataset_freiburg1_xyz";

class TwoViewGeometryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    loader::TUMLoader loader;
    ASSERT_TRUE(loader.load(kDatasetPath));
    dataset_ = loader.dataset();

    // Load first two images
    std::string img1_path =
        dataset_.base_path + "/" + dataset_.rgb_frames[0].relative_path;
    std::string img2_path =
        dataset_.base_path + "/" + dataset_.rgb_frames[10].relative_path;

    img1_ = cv::imread(img1_path, cv::IMREAD_GRAYSCALE);
    img2_ = cv::imread(img2_path, cv::IMREAD_GRAYSCALE);
    ASSERT_FALSE(img1_.empty());
    ASSERT_FALSE(img2_.empty());

    // Extract and match features
    FeatureExtractorCPU extractor(1000);
    r1_ = extractor.extract(img1_);
    r2_ = extractor.extract(img2_);

    FeatureMatcherCPU matcher;
    matches_ = matcher.match(r1_.descriptors, r2_.descriptors, 0.75f);
  }

  loader::TUMDataset dataset_;
  cv::Mat img1_, img2_;
  ExtractionResult r1_, r2_;
  std::vector<cv::DMatch> matches_;
};

TEST_F(TwoViewGeometryTest, EstimateFromRealImages) {
  ASSERT_GT(matches_.size(), 50u);

  TwoViewGeometry tvg;
  auto result =
      tvg.estimate(r1_.keypoints, r2_.keypoints, matches_, dataset_.intrinsics);

  EXPECT_TRUE(result.success);
  EXPECT_GT(result.num_inliers, 20);
  EXPECT_GT(result.points_3d.size(), 10u);

  // R should be close to identity for small baseline (fr1/xyz is slow motion)
  // Check that rotation angle is small (< 10 degrees)
  double trace = result.R.trace();
  double angle_rad =
      std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
  double angle_deg = angle_rad * 180.0 / M_PI;
  EXPECT_LT(angle_deg, 10.0)
      << "Rotation angle too large for consecutive frames: " << angle_deg
      << " degrees";

  // Translation should be unit norm
  EXPECT_NEAR(result.t.norm(), 1.0, 1e-6);

  // All 3D points should have positive depth
  for (const auto& pt : result.points_3d) {
    EXPECT_GT(pt.z(), 0.0) << "Triangulated point has negative depth";
  }

  std::cout << "Two-view geometry:\n"
            << "  Matches: " << matches_.size() << "\n"
            << "  Inliers: " << result.num_inliers << "\n"
            << "  Triangulated points: " << result.points_3d.size() << "\n"
            << "  Rotation angle: " << angle_deg << " deg\n"
            << "  Translation: [" << result.t.transpose() << "]\n";
}

TEST_F(TwoViewGeometryTest, TooFewMatches) {
  // Create a very small set of matches
  std::vector<cv::DMatch> few_matches(
      matches_.begin(),
      matches_.begin() + std::min(size_t(5), matches_.size()));

  TwoViewGeometry tvg;
  auto result = tvg.estimate(r1_.keypoints, r2_.keypoints, few_matches,
                             dataset_.intrinsics);

  EXPECT_FALSE(result.success);
}

TEST_F(TwoViewGeometryTest, RotationMatrixValid) {
  TwoViewGeometry tvg;
  auto result =
      tvg.estimate(r1_.keypoints, r2_.keypoints, matches_, dataset_.intrinsics);

  ASSERT_TRUE(result.success);

  // R should be a valid rotation matrix: R^T R = I, det(R) = 1
  Eigen::Matrix3d RtR = result.R.transpose() * result.R;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double expected = (i == j) ? 1.0 : 0.0;
      EXPECT_NEAR(RtR(i, j), expected, 1e-6)
          << "R^T R not identity at (" << i << "," << j << ")";
    }
  }
  // Compute determinant manually (avoids Eigen template instantiation issues
  // with EIGEN_MAX_ALIGN_BYTES mismatch between cuda and non-cuda TUs)
  double det =
      result.R(0, 0) *
          (result.R(1, 1) * result.R(2, 2) - result.R(1, 2) * result.R(2, 1)) -
      result.R(0, 1) *
          (result.R(1, 0) * result.R(2, 2) - result.R(1, 2) * result.R(2, 0)) +
      result.R(0, 2) *
          (result.R(1, 0) * result.R(2, 1) - result.R(1, 1) * result.R(2, 0));
  EXPECT_NEAR(det, 1.0, 1e-6);
}

}  // namespace
}  // namespace frontend
}  // namespace substral
