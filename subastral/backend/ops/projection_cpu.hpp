#pragma once

namespace substral {
namespace backend {
namespace ops {

// Project a 3D point through a BAL camera model.
//
// Camera parameter layout:
//   cam[0..2] = angle-axis rotation
//   cam[3..5] = translation
//   cam[6]    = focal length
//   cam[7]    = k1 (radial distortion)
//   cam[8]    = k2 (radial distortion)
//
// Output: prediction[0..1] = projected 2D pixel coordinates.
//
// Implementation in projection_cpu.cpp.
void project_cpu(const double* camera, const double* point, double* prediction);

}  // namespace ops
}  // namespace backend
}  // namespace substral
