#include "subastral/backend/common.h"
#include "subastral/backend/ops/jacobians.h"
#include "subastral/backend/ops/project_with_jacobian_cpu.hpp"

namespace substral {
namespace backend {
namespace ops {

double computeResidualsAndJacobiansCPU(backend::BAProblem& problem,
                                       double* residuals, double* J_cameras,
                                       double* J_points) {
  double total_sq_error = 0.0;

  for (int i = 0; i < static_cast<int>(problem.observations.size()); ++i) {
    auto& observation = problem.observations[i];
    const double* cam = problem.cameras[observation->get_camera_id()]->data();
    const double* point = problem.points[observation->get_point_id()]->data();

    double predicted[2];
    double* J_cam = &J_cameras[i * 18];
    double* J_pt = &J_points[i * 6];

    projectWithJacobianCPU(cam, point, predicted, J_cam, J_pt);

    // Residual = predicted - observed
    residuals[i * 2 + 0] = predicted[0] - observation->data()[0];
    residuals[i * 2 + 1] = predicted[1] - observation->data()[1];

    total_sq_error += residuals[i * 2] * residuals[i * 2] +
                      residuals[i * 2 + 1] * residuals[i * 2 + 1];
  }

  return total_sq_error;
}

}  // namespace ops
}  // namespace backend
}  // namespace substral
