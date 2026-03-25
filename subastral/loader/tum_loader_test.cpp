#include "subastral/loader/tum_loader.h"

#include <cmath>
#include <iostream>

#include "gtest/gtest.h"

namespace substral {
namespace loader {
namespace {

const std::string kFr1XyzPath =
    "/data/se/tum_datasets/rgbd_dataset_freiburg1_xyz";

// Test loading the fr1/xyz dataset
TEST(TUMLoaderTest, LoadFr1Xyz) {
  TUMLoader loader;
  ASSERT_TRUE(loader.load(kFr1XyzPath));

  const auto& ds = loader.dataset();

  // fr1/xyz has 798 RGB frames
  EXPECT_EQ(ds.rgb_frames.size(), 798u);
  EXPECT_GT(ds.ground_truth.size(), 0u);

  // Should detect freiburg1
  EXPECT_EQ(ds.sensor_id, 1);

  // Intrinsics should be ROS defaults (TUM recommendation: no undistortion)
  EXPECT_NEAR(ds.intrinsics.fx, 525.0, 1e-3);
  EXPECT_NEAR(ds.intrinsics.fy, 525.0, 1e-3);
  EXPECT_NEAR(ds.intrinsics.cx, 319.5, 1e-3);
  EXPECT_NEAR(ds.intrinsics.cy, 239.5, 1e-3);

  // No distortion — images are already corrected by the OpenNI driver
  EXPECT_TRUE(ds.intrinsics.isUndistorted());

  loader.printStats();
}

// Test that timestamps are monotonically increasing
TEST(TUMLoaderTest, TimestampsMonotonic) {
  TUMLoader loader;
  ASSERT_TRUE(loader.load(kFr1XyzPath));

  const auto& frames = loader.dataset().rgb_frames;
  for (size_t i = 1; i < frames.size(); ++i) {
    EXPECT_GT(frames[i].timestamp, frames[i - 1].timestamp)
        << "RGB timestamps not monotonic at index " << i;
  }

  const auto& gt = loader.dataset().ground_truth;
  for (size_t i = 1; i < gt.size(); ++i) {
    EXPECT_GT(gt[i].timestamp, gt[i - 1].timestamp)
        << "Ground truth timestamps not monotonic at index " << i;
  }
}

// Test that ground truth quaternions are unit
TEST(TUMLoaderTest, GroundTruthQuaternionsUnit) {
  TUMLoader loader;
  ASSERT_TRUE(loader.load(kFr1XyzPath));

  for (const auto& pose : loader.dataset().ground_truth) {
    double qnorm = std::sqrt(pose.qx * pose.qx + pose.qy * pose.qy +
                             pose.qz * pose.qz + pose.qw * pose.qw);
    EXPECT_NEAR(qnorm, 1.0, 1e-3)
        << "Ground truth quaternion not unit at t=" << pose.timestamp;
  }
}

// Test that image paths are valid relative paths
TEST(TUMLoaderTest, ImagePathsValid) {
  TUMLoader loader;
  ASSERT_TRUE(loader.load(kFr1XyzPath));

  const auto& frames = loader.dataset().rgb_frames;
  ASSERT_FALSE(frames.empty());

  // First few paths should start with "rgb/"
  for (size_t i = 0; i < std::min(frames.size(), size_t(5)); ++i) {
    EXPECT_EQ(frames[i].relative_path.substr(0, 4), "rgb/")
        << "Image path doesn't start with 'rgb/': " << frames[i].relative_path;
  }
}

// Test CameraIntrinsics helper methods
TEST(TUMLoaderTest, CameraIntrinsicsHelpers) {
  TUMLoader loader;
  ASSERT_TRUE(loader.load(kFr1XyzPath));

  const auto& intrinsics = loader.dataset().intrinsics;

  // Camera matrix should be 3x3
  cv::Mat K = intrinsics.cameraMatrix();
  EXPECT_EQ(K.rows, 3);
  EXPECT_EQ(K.cols, 3);
  EXPECT_NEAR(K.at<double>(0, 0), intrinsics.fx, 1e-10);
  EXPECT_NEAR(K.at<double>(1, 1), intrinsics.fy, 1e-10);
  EXPECT_NEAR(K.at<double>(0, 2), intrinsics.cx, 1e-10);
  EXPECT_NEAR(K.at<double>(1, 2), intrinsics.cy, 1e-10);

  // Distortion coefficients should be 5x1, all zero
  cv::Mat d = intrinsics.distCoeffs();
  EXPECT_EQ(d.rows, 5);
  EXPECT_EQ(d.cols, 1);
  EXPECT_NEAR(d.at<double>(0), 0.0, 1e-10);
}

// Test loading a nonexistent path
TEST(TUMLoaderTest, NonexistentPath) {
  TUMLoader loader;
  EXPECT_FALSE(loader.load("/nonexistent/path"));
}

}  // namespace
}  // namespace loader
}  // namespace substral
