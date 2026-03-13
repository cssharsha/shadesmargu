#pragma once

#include <cuda_runtime.h>
#include <math.h>

namespace substral {
namespace backend {
namespace solver {
namespace gpu {

// =============================================================================
// GPU Loss Function Evaluators
// =============================================================================
//
// Device-side loss function evaluation. These are standalone __device__
// functions rather than structs to avoid vtable issues on the GPU.
//
// Each loss type provides:
//   rho(s, param):    loss value for squared residual norm s
//   weight(s, param): IRLS weight = rho'(s), used to scale J and r
//
// The loss type is selected via an enum, and a single dispatcher function
// evaluates the appropriate loss. This avoids polymorphism on the GPU.
//
// =============================================================================

enum class LossType : int {
  TRIVIAL = 0,
  HUBER = 1,
  CAUCHY = 2,
};

// Trivial: rho(s) = s, weight = 1
__device__ inline double trivialRho(double s) { return s; }
__device__ inline double trivialWeight(double /*s*/) { return 1.0; }

// Huber: param = delta
__device__ inline double huberRho(double s, double delta) {
  double r = sqrt(s);
  if (r <= delta) {
    return s;
  } else {
    return 2.0 * delta * r - delta * delta;
  }
}

__device__ inline double huberWeight(double s, double delta) {
  if (s < 1e-30) return 1.0;
  double r = sqrt(s);
  if (r <= delta) {
    return 1.0;
  } else {
    return delta / r;
  }
}

// Cauchy: param = c, c_sq = c*c
__device__ inline double cauchyRho(double s, double c_sq) {
  return c_sq * log(1.0 + s / c_sq);
}

__device__ inline double cauchyWeight(double s, double c_sq) {
  return c_sq / (c_sq + s);
}

// Dispatcher: evaluate rho given loss type and parameter
__device__ inline double evalRho(LossType type, double s, double param) {
  switch (type) {
    case LossType::HUBER:
      return huberRho(s, param);
    case LossType::CAUCHY:
      return cauchyRho(s, param * param);  // param = c, need c^2
    case LossType::TRIVIAL:
    default:
      return trivialRho(s);
  }
}

// Dispatcher: evaluate weight given loss type and parameter
__device__ inline double evalWeight(LossType type, double s, double param) {
  switch (type) {
    case LossType::HUBER:
      return huberWeight(s, param);
    case LossType::CAUCHY:
      return cauchyWeight(s, param * param);  // param = c, need c^2
    case LossType::TRIVIAL:
    default:
      return trivialWeight(s);
  }
}

}  // namespace gpu
}  // namespace solver
}  // namespace backend
}  // namespace substral
