#include "subastral/frontend/imu_initializer.hpp"

#include <iostream>

namespace substral {
namespace frontend {

bool ImuInitializer::initialize(
    const std::vector<loader::ImuMeasurement>& /*imu_data*/,
    std::vector<Frame>& /*frames*/, const CameraIntrinsics& /*intrinsics*/,
    Map& /*map*/, int& /*out_idx1*/, int& /*out_idx2*/) {
  std::cerr << "ImuInitializer: NOT YET IMPLEMENTED. "
            << "Falling back to monocular initialization." << std::endl;
  return false;
}

}  // namespace frontend
}  // namespace substral
