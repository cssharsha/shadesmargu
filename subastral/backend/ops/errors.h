#pragma once

#include "subastral/backend/common.h"

namespace substral {
namespace backend {
namespace ops {

// CPU projection error computation.
// Computes sum of squared reprojection errors across all observations.
double computeProjectionErrorCPU(backend::BAProblem& problem);

// GPU projection error computation.
// Same as CPU version but runs on GPU.
double computeProjectionErrorGPU(backend::BAProblem& problem);

}  // namespace ops
}  // namespace backend
}  // namespace substral
