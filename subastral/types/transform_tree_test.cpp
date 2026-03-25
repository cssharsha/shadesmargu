#include "subastral/types/transform_tree.h"

#include <Eigen/Geometry>
#include <cmath>

#include "gtest/gtest.h"

namespace substral {
namespace {

// Helper: check two 4x4 matrices are approximately equal.
void expectNear(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b,
                double tol = 1e-10) {
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      EXPECT_NEAR(a(r, c), b(r, c), tol) << "at (" << r << "," << c << ")";
}

// Helper: build a 4x4 SE(3) transform from rotation angle (about Z) and
// translation.
Eigen::Matrix4d makeTransform(double angle_deg, double tx, double ty,
                              double tz) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  double rad = angle_deg * M_PI / 180.0;
  T.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T(0, 3) = tx;
  T(1, 3) = ty;
  T(2, 3) = tz;
  return T;
}

TEST(TransformTreeTest, IdentityLookup) {
  TransformTree tree;
  tree.setStatic("world", "optical", Eigen::Matrix4d::Identity());
  expectNear(tree.lookup("world", "world"), Eigen::Matrix4d::Identity());
  expectNear(tree.lookup("optical", "optical"), Eigen::Matrix4d::Identity());
}

TEST(TransformTreeTest, DirectLookup) {
  TransformTree tree;
  Eigen::Matrix4d T = makeTransform(30.0, 1.0, 2.0, 3.0);
  tree.setStatic("world", "optical", T);

  // Forward: T_world_optical
  expectNear(tree.lookup("world", "optical"), T);

  // Inverse: T_optical_world = inv(T_world_optical)
  expectNear(tree.lookup("optical", "world"), T.inverse());
}

TEST(TransformTreeTest, ChainedLookup) {
  TransformTree tree;
  Eigen::Matrix4d T_wo = makeTransform(10.0, 0.1, 0.0, 0.0);
  Eigen::Matrix4d T_oi = makeTransform(5.0, 0.05, 0.01, 0.0);

  tree.setStatic("world", "optical", T_wo);
  tree.setStatic("optical", "imu", T_oi);

  // T_world_imu = T_world_optical * T_optical_imu
  Eigen::Matrix4d expected = T_wo * T_oi;
  expectNear(tree.lookup("world", "imu"), expected);

  // Inverse chain
  expectNear(tree.lookup("imu", "world"), expected.inverse());
}

TEST(TransformTreeTest, SiblingLookup) {
  TransformTree tree;
  Eigen::Matrix4d T_wa = makeTransform(15.0, 1.0, 0.0, 0.0);
  Eigen::Matrix4d T_wb = makeTransform(-20.0, 0.0, 2.0, 0.0);

  tree.setStatic("world", "A", T_wa);
  tree.setStatic("world", "B", T_wb);

  // T_A_B = inv(T_world_A) * T_world_B
  Eigen::Matrix4d expected = T_wa.inverse() * T_wb;
  expectNear(tree.lookup("A", "B"), expected);
  expectNear(tree.lookup("B", "A"), expected.inverse());
}

TEST(TransformTreeTest, UnknownFrameThrows) {
  TransformTree tree;
  tree.setStatic("world", "optical", Eigen::Matrix4d::Identity());

  EXPECT_THROW(tree.lookup("world", "nonexistent"), TransformException);
  EXPECT_THROW(tree.lookup("nonexistent", "world"), TransformException);
}

TEST(TransformTreeTest, DisconnectedTreesThrow) {
  TransformTree tree;
  tree.setStatic("world", "optical", Eigen::Matrix4d::Identity());
  tree.setStatic("other_root", "other_child", Eigen::Matrix4d::Identity());

  EXPECT_THROW(tree.lookup("optical", "other_child"), TransformException);
}

TEST(TransformTreeTest, DuplicateParentThrows) {
  TransformTree tree;
  tree.setStatic("world", "optical", Eigen::Matrix4d::Identity());

  // Try to reparent "optical" to a different parent
  EXPECT_THROW(
      tree.setStatic("other", "optical", Eigen::Matrix4d::Identity()),
      TransformException);
}

TEST(TransformTreeTest, CycleDetection) {
  TransformTree tree;
  tree.setStatic("A", "B", Eigen::Matrix4d::Identity());

  // B→A would create A→B→A cycle
  EXPECT_THROW(tree.setStatic("B", "A", Eigen::Matrix4d::Identity()),
               TransformException);
}

TEST(TransformTreeTest, UpdateExistingTransform) {
  TransformTree tree;
  Eigen::Matrix4d T1 = makeTransform(10.0, 1.0, 0.0, 0.0);
  Eigen::Matrix4d T2 = makeTransform(20.0, 2.0, 0.0, 0.0);

  tree.setStatic("world", "optical", T1);
  expectNear(tree.lookup("world", "optical"), T1);

  // Re-setting with same parent should update the transform
  tree.setStatic("world", "optical", T2);
  expectNear(tree.lookup("world", "optical"), T2);
}

TEST(TransformTreeTest, HasFrame) {
  TransformTree tree;
  EXPECT_FALSE(tree.hasFrame("world"));

  tree.setStatic("world", "optical", Eigen::Matrix4d::Identity());
  EXPECT_TRUE(tree.hasFrame("world"));
  EXPECT_TRUE(tree.hasFrame("optical"));
  EXPECT_FALSE(tree.hasFrame("imu"));
}

TEST(TransformTreeTest, FramesList) {
  TransformTree tree;
  tree.setStatic("world", "optical", Eigen::Matrix4d::Identity());
  tree.setStatic("optical", "imu", Eigen::Matrix4d::Identity());

  auto f = tree.frames();
  EXPECT_EQ(f.size(), 3u);  // world, optical, imu
}

TEST(TransformTreeTest, NonThrowingLookup) {
  TransformTree tree;
  tree.setStatic("world", "optical", Eigen::Matrix4d::Identity());

  bool ok = false;
  auto T = tree.lookup("world", "optical", ok);
  EXPECT_TRUE(ok);
  expectNear(T, Eigen::Matrix4d::Identity());

  T = tree.lookup("world", "nonexistent", ok);
  EXPECT_FALSE(ok);
  expectNear(T, Eigen::Matrix4d::Identity());
}

TEST(TransformTreeTest, ZUpConvention) {
  // Verify the optical→world rotation converts correctly:
  //   cam X-right  → world X-right
  //   cam Y-down   → world -Z (down)
  //   cam Z-forward → world Y-forward
  TransformTree tree;
  Eigen::Matrix4d T_world_optical = Eigen::Matrix4d::Identity();
  // clang-format off
  T_world_optical.block<3, 3>(0, 0) << 1,  0,  0,
                                        0,  0,  1,
                                        0, -1,  0;
  // clang-format on
  tree.setStatic("world", "optical", T_world_optical);

  Eigen::Matrix4d T = tree.lookup("world", "optical");

  // Camera Z-forward [0,0,1] → world Y-forward [0,1,0]
  Eigen::Vector4d cam_forward(0, 0, 1, 1);
  Eigen::Vector4d world_result = T * cam_forward;
  EXPECT_NEAR(world_result.x(), 0.0, 1e-10);
  EXPECT_NEAR(world_result.y(), 1.0, 1e-10);
  EXPECT_NEAR(world_result.z(), 0.0, 1e-10);

  // Camera Y-down [0,1,0] → world -Z [0,0,-1]
  Eigen::Vector4d cam_down(0, 1, 0, 1);
  world_result = T * cam_down;
  EXPECT_NEAR(world_result.x(), 0.0, 1e-10);
  EXPECT_NEAR(world_result.y(), 0.0, 1e-10);
  EXPECT_NEAR(world_result.z(), -1.0, 1e-10);

  // Camera X-right [1,0,0] → world X-right [1,0,0]
  Eigen::Vector4d cam_right(1, 0, 0, 1);
  world_result = T * cam_right;
  EXPECT_NEAR(world_result.x(), 1.0, 1e-10);
  EXPECT_NEAR(world_result.y(), 0.0, 1e-10);
  EXPECT_NEAR(world_result.z(), 0.0, 1e-10);
}

}  // namespace
}  // namespace substral
