#pragma once

#include <cuda_runtime.h>

#include <cmath>

namespace substral {
namespace backend {
namespace ops {
namespace device {

__device__ inline void rotatePoint(const double* w, const double* P,
                                   double* P_out) {
  double theta = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
  if (theta < 1e-10) {
    P_out[0] = P[0] + (w[1] * P[2] - w[2] * P[1]);
    P_out[1] = P[1] + (w[2] * P[0] - w[0] * P[2]);
    P_out[2] = P[2] + (w[0] * P[1] - w[1] * P[0]);
    return;
  }

  double k[3] = {w[0] / theta, w[1] / theta, w[2] / theta};
  double cos_theta = cos(theta);
  double sin_theta = sin(theta);

  double k_dot_p = k[0] * P[0] + k[1] * P[1] + k[2] * P[2];

  double k_cross_p[3];
  k_cross_p[0] = k[1] * P[2] - k[2] * P[1];
  k_cross_p[1] = k[2] * P[0] - k[0] * P[2];
  k_cross_p[2] = k[0] * P[1] - k[1] * P[0];

  double one_minus_cos = 1.0 - cos_theta;

  P_out[0] = P[0] * cos_theta + k_cross_p[0] * sin_theta +
             k[0] * k_dot_p * one_minus_cos;
  P_out[1] = P[1] * cos_theta + k_cross_p[1] * sin_theta +
             k[1] * k_dot_p * one_minus_cos;
  P_out[2] = P[2] * cos_theta + k_cross_p[2] * sin_theta +
             k[2] * k_dot_p * one_minus_cos;
}

// This does only pin hole projection. BAL format my firend.
__device__ inline void projectPoint(const double* cam, const double* pt,
                                    double* uv_out) {
  double p_rotated[3];
  rotatePoint(cam, pt, p_rotated);

  // Translation
  double p_c[3] = {p_rotated[0] + cam[3], p_rotated[1] + cam[4],
                   p_rotated[2] + cam[5]};

  // Perspective Divide (Snavely: -P / P.z)
  // This is according to the BAL format,
  // might change if its a different format
  double xp = -p_c[0] / p_c[2];
  double yp = -p_c[1] / p_c[2];

  // Distortion
  double r2 = xp * xp + yp * yp;
  double r4 = r2 * r2;
  double k1 = cam[7];
  double k2 = cam[8];
  double dist = 1.0 + k1 * r2 + k2 * r4;

  // Focal scaling
  double f = cam[6];
  uv_out[0] = f * dist * xp;
  uv_out[1] = f * dist * yp;
}

}  // namespace device
}  // namespace ops
}  // namespace backend
}  // namespace substral
