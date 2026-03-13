#include "subastral/backend/common.h"

#include <gtest/gtest.h>

#include <memory>

namespace substral {
namespace backend {

class CommonTest : public ::testing::Test {
 protected:
  void SetUp() override {
    memory_map_ = std::make_shared<FactorGraphMemoryMap>();
  }

  std::shared_ptr<FactorGraphMemoryMap> memory_map_;
};

TEST_F(CommonTest, CameraDataTest) {
  int camera_id = 1;
  Camera camera(camera_id, memory_map_);

  // Verify type and id
  EXPECT_EQ(camera.get_type(), GraphEntity::OBSERVER);
  EXPECT_EQ(camera.get_id(), camera_id);
  EXPECT_EQ(camera.get_stride(), 9);

  // Set some dummy data
  double* data = camera.mutable_data();
  for (int i = 0; i < 9; ++i) {
    data[i] = static_cast<double>(i) * 1.5;
  }

  // Verify data is properly set in the memory map
  EXPECT_EQ(memory_map_->observers.size(), 9);
  EXPECT_EQ(memory_map_->observer_indices[camera_id], 0);

  const double* const_data = camera.data();
  for (int i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(const_data[i], static_cast<double>(i) * 1.5);
    EXPECT_DOUBLE_EQ(memory_map_->observers[i], static_cast<double>(i) * 1.5);
  }
}

TEST_F(CommonTest, PointDataTest) {
  int point_id = 10;
  Point point(point_id, memory_map_);

  // Verify type and id
  EXPECT_EQ(point.get_type(), GraphEntity::SCENE_POINT);
  EXPECT_EQ(point.get_id(), point_id);
  EXPECT_EQ(point.get_stride(), 3);

  // Set some dummy data
  double* data = point.mutable_data();
  data[0] = 10.0;
  data[1] = 20.0;
  data[2] = 30.0;

  // Verify data is properly set in the memory map
  EXPECT_EQ(memory_map_->scene_points.size(), 3);
  EXPECT_EQ(memory_map_->scene_point_indices[point_id], 0);

  const double* const_data = point.data();
  EXPECT_DOUBLE_EQ(const_data[0], 10.0);
  EXPECT_DOUBLE_EQ(const_data[1], 20.0);
  EXPECT_DOUBLE_EQ(const_data[2], 30.0);

  EXPECT_DOUBLE_EQ(memory_map_->scene_points[0], 10.0);
  EXPECT_DOUBLE_EQ(memory_map_->scene_points[1], 20.0);
  EXPECT_DOUBLE_EQ(memory_map_->scene_points[2], 30.0);
}

TEST_F(CommonTest, ObservationDataTest) {
  int obs_id = 100;
  int cam_id = 1;
  int pt_id = 10;
  Observation observation(obs_id, cam_id, pt_id, memory_map_);

  // Verify type and id
  EXPECT_EQ(observation.get_type(), GraphEntity::OBSERVATION);
  EXPECT_EQ(observation.get_id(), obs_id);
  EXPECT_EQ(observation.get_stride(), 2);
  EXPECT_EQ(observation.get_camera_id(), cam_id);
  EXPECT_EQ(observation.get_point_id(), pt_id);

  // Observation data is immutable via mutable_data(), but we can check the
  // memory map However, the class design implies we should be able to set it
  // somehow, but for now let's just manually populate the memory map to verify
  // data() reads correctly.

  // Note: Observation::mutable_data() throws.
  EXPECT_THROW(observation.mutable_data(), std::runtime_error);

  // Manually set data in memory map for testing read
  memory_map_->observations[0] = 100.5;
  memory_map_->observations[1] = 200.5;

  const double* const_data = observation.data();
  EXPECT_DOUBLE_EQ(const_data[0], 100.5);
  EXPECT_DOUBLE_EQ(const_data[1], 200.5);
}

TEST_F(CommonTest, MultipleEntitiesTest) {
  Camera cam1(1, memory_map_);
  Camera cam2(2, memory_map_);

  EXPECT_EQ(memory_map_->observers.size(), 18);
  EXPECT_EQ(memory_map_->observer_indices[1], 0);
  EXPECT_EQ(memory_map_->observer_indices[2], 9);

  double* data1 = cam1.mutable_data();
  double* data2 = cam2.mutable_data();

  data1[0] = 1.1;
  data2[0] = 2.2;

  EXPECT_DOUBLE_EQ(memory_map_->observers[0], 1.1);
  EXPECT_DOUBLE_EQ(memory_map_->observers[9], 2.2);
}

}  // namespace backend
}  // namespace substral
