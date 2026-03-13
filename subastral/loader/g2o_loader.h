#pragma once

#include <memory>
#include <string>
#include <vector>

#include "subastral/backend/common.h"

namespace substral {
namespace loader {

// =============================================================================
// g2o file loader for 3D pose-graph SLAM
// =============================================================================
//
// Parses g2o files containing:
//   VERTEX_SE3:QUAT id x y z qx qy qz qw
//   EDGE_SE3:QUAT id1 id2 dx dy dz dqx dqy dqz dqw <21 upper-tri info>
//
// Quaternion convention: Hamilton (qx, qy, qz, qw) with w last.
//
// The 21 upper-triangular entries of the 6×6 information matrix are
// stored row-major:
//   I(0,0) I(0,1) I(0,2) I(0,3) I(0,4) I(0,5)
//          I(1,1) I(1,2) I(1,3) I(1,4) I(1,5)
//                 I(2,2) I(2,3) I(2,4) I(2,5)
//                        I(3,3) I(3,4) I(3,5)
//                               I(4,4) I(4,5)
//                                      I(5,5)
//
// g2o convention for info matrix ordering is (translation, rotation) but
// our internal convention is (rotation, translation). The loader handles
// this reordering.
//
// =============================================================================

class G2OLoader {
 public:
  explicit G2OLoader(const std::string& filename);

  // Load into a PoseGraphProblem (convenience overload)
  bool Load(backend::PoseGraphProblem& problem);

  // Load into raw components
  bool Load(std::shared_ptr<backend::FactorGraphMemoryMap> memory_map,
            std::vector<std::shared_ptr<backend::Pose>>& poses,
            std::vector<std::shared_ptr<backend::PoseEdge>>& edges);

 private:
  std::string filename_;
};

}  // namespace loader
}  // namespace substral
