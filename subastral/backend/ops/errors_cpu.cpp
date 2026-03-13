#include "subastral/backend/common.h"
#include "subastral/backend/ops/errors.h"
#include "subastral/backend/ops/projection_cpu.hpp"

namespace substral {
namespace backend {
namespace ops {

double computeProjectionErrorCPU(backend::BAProblem& problem) {
  double total_sq_error = 0;
  for (auto& observation : problem.observations) {
    auto cam = problem.cameras[observation->get_camera_id()]->data();
    auto point = problem.points[observation->get_point_id()]->data();

    double prediction[2];
    backend::ops::project_cpu(cam, point, prediction);

    double residual[2];
    residual[0] = prediction[0] - observation->data()[0];
    residual[1] = prediction[1] - observation->data()[1];

    total_sq_error += residual[0] * residual[0] + residual[1] * residual[1];
  }
  return total_sq_error;
}

}  // namespace ops
}  // namespace backend
}  // namespace substral
