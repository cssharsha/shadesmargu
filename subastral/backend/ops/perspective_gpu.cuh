#pragma once

#include <cuda_runtime.h>

namespace substral {
namespace backend {
namespace ops {
namespace device {

// =============================================================================
// Perspective Divide with Analytical Jacobian
// =============================================================================
//
// Given a point in camera coordinates P_cam = (X, Y, Z), the BAL format
// uses the Snavely projection convention:
//
//   u = -X / Z
//   v = -Y / Z
//
// (The negation is because the BAL camera looks down -Z, with the image
// plane behind the camera center.)
//
// The Jacobian d(u,v)/dP_cam is a 2×3 matrix:
//
//   ∂u/∂X = -1/Z
//   ∂u/∂Y =  0
//   ∂u/∂Z =  X/Z²     (since u = -X/Z, ∂u/∂Z = X/Z²)
//
//   ∂v/∂X =  0
//   ∂v/∂Y = -1/Z
//   ∂v/∂Z =  Y/Z²     (since v = -Y/Z, ∂v/∂Z = Y/Z²)
//
// In matrix form:
//
//   d(u,v)/dP_cam = | -1/Z    0    X/Z² |
//                   |   0   -1/Z   Y/Z² |
//
// Or equivalently, using u = -X/Z and v = -Y/Z:
//
//   d(u,v)/dP_cam = | -1/Z    0    -u/Z |
//                   |   0   -1/Z   -v/Z |
//
// =============================================================================

__device__ inline void perspectiveDivideWithJacobian(
    const double* P_cam,    // [3] point in camera frame (X, Y, Z)
    double* uv,             // [2] normalized image coordinates (u, v)
    double* duv_dPcam       // [6] row-major 2×3: d(u,v)/dP_cam
) {
  double X = P_cam[0];
  double Y = P_cam[1];
  double Z = P_cam[2];

  double inv_Z = 1.0 / Z;
  double inv_Z2 = inv_Z * inv_Z;

  // Projection (BAL convention: negate)
  uv[0] = -X * inv_Z;
  uv[1] = -Y * inv_Z;

  // Jacobian d(u,v)/d(X,Y,Z)  — 2×3 row-major
  // Row 0: du/dX, du/dY, du/dZ
  duv_dPcam[0] = -inv_Z;
  duv_dPcam[1] = 0.0;
  duv_dPcam[2] = X * inv_Z2;

  // Row 1: dv/dX, dv/dY, dv/dZ
  duv_dPcam[3] = 0.0;
  duv_dPcam[4] = -inv_Z;
  duv_dPcam[5] = Y * inv_Z2;
}

}  // namespace device
}  // namespace ops
}  // namespace backend
}  // namespace substral
