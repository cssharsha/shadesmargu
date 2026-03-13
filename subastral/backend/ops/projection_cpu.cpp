#include "subastral/backend/ops/projection_cpu.hpp"

#include <Eigen/Dense>

namespace substral {
namespace backend {
namespace ops {

void project_cpu(const double* camera, const double* point,
                 double* prediction) {
  // Angle-axis rotation
  Eigen::Map<const Eigen::Vector3d> omega(camera);
  // Translation
  Eigen::Map<const Eigen::Vector3d> t(camera + 3);
  double f = camera[6];
  double k1 = camera[7];
  double k2 = camera[8];

  Eigen::Map<const Eigen::Vector3d> P_world(point);

  Eigen::Vector3d P_cam;
  double theta = omega.norm();
  if (theta > 1e-6) {
    Eigen::Vector3d axis = omega / theta;
    P_cam = Eigen::AngleAxis<double>(theta, axis) * P_world + t;
  } else {
    P_cam = P_world + omega.cross(P_world) + t;
  }

  // Project to image plane
  double u = -P_cam[0] / P_cam[2];
  double v = -P_cam[1] / P_cam[2];

  // Distortion
  double r2 = u * u + v * v;
  double r4 = r2 * r2;
  double factor = 1.0 + k1 * r2 + k2 * r4;

  prediction[0] = f * factor * u;
  prediction[1] = f * factor * v;
}

}  // namespace ops
}  // namespace backend
}  // namespace substral
