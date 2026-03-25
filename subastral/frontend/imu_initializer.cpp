#include "subastral/frontend/imu_initializer.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

#include "subastral/backend/lie/so3.hpp"

namespace substral {
namespace frontend {

ImuInitializer::InitResult ImuInitializer::initialize(
    const std::vector<loader::ImuMeasurement>& imu_data,
    const std::vector<loader::ImageEntry>& rgb_frames,
    const CameraIntrinsics& intrinsics,
    std::shared_ptr<TransformTree> tf_tree) {
  InitResult result;

  if (imu_data.empty() || rgb_frames.size() < 10) {
    std::cerr << "ImuInitializer: insufficient data" << std::endl;
    return result;
  }

  // Step 1: Estimate gravity direction from initial static IMU data
  auto gravity = gravity_init_.estimate(imu_data);
  if (!gravity.success) {
    std::cerr << "ImuInitializer: gravity estimation failed" << std::endl;
    return result;
  }

  // Register world→imu0 in the transform tree (gravity alignment)
  Eigen::Matrix4d T_world_imu = Eigen::Matrix4d::Identity();
  T_world_imu.block<3, 3>(0, 0) = gravity.R_world_imu;
  tf_tree->setStatic("world", "imu0", T_world_imu);

  std::cout << "  Gravity aligned: R_world_imu registered" << std::endl;

  // Step 2: Search for frame pair with sufficient rotation (from gyro).
  // delta_p from pre-integration is NOT reliable for displacement without
  // gravity subtraction and velocity estimation. Use gyro rotation angle
  // as the selection criterion instead — sufficient rotation implies
  // sufficient parallax for triangulation.
  constexpr double kMinRotationDeg = 2.0;

  int idx1 = 0;
  double ts1 = rgb_frames[idx1].timestamp;

  for (int gap = 5; gap <= config_.max_frame_gap &&
                     idx1 + gap < static_cast<int>(rgb_frames.size());
       gap += 5) {
    int idx2 = idx1 + gap;
    double ts2 = rgb_frames[idx2].timestamp;

    // Pre-integrate IMU between frame pair (gyro rotation is reliable)
    ImuPreintegrator preint;
    preint.integrateRange(imu_data, ts1, ts2);
    auto pi = preint.result();

    double rot_deg =
        backend::lie::logSO3(pi.delta_R).norm() * 180.0 / M_PI;

    std::cout << "  IMU pair (" << idx1 << "," << idx2 << "): "
              << "dt=" << std::fixed << std::setprecision(3) << pi.dt << "s"
              << " rot=" << std::setprecision(2) << rot_deg << "deg"
              << std::endl;

    if (rot_deg >= kMinRotationDeg) {
      result.idx1 = idx1;
      result.idx2 = idx2;
      result.R_imu_12 = pi.delta_R;
      result.success = true;

      std::cout << "  Selected pair (" << idx1 << "," << idx2
                << ") with " << std::setprecision(2) << rot_deg
                << " deg rotation" << std::endl;
      return result;
    }
  }

  std::cerr << "ImuInitializer: no frame pair with sufficient rotation"
            << std::endl;
  return result;
}

}  // namespace frontend
}  // namespace substral
