#pragma once

#include <cuda_runtime.h>

namespace substral {
namespace backend {
namespace ops {
namespace device {

// =============================================================================
// Radial Distortion + Focal Scaling with Analytical Jacobian
// =============================================================================
//
// Given normalized image coordinates (u, v) from the perspective divide,
// the BAL distortion model applies:
//
//   r² = u² + v²
//   r⁴ = r²·r²
//   D  = 1 + k1·r² + k2·r⁴          (distortion factor)
//
//   px = f · D · u                    (final pixel x)
//   py = f · D · v                    (final pixel y)
//
// The camera intrinsics are: cam[6] = f, cam[7] = k1, cam[8] = k2.
//
// We need the Jacobian of (px, py) w.r.t. (u, v, f, k1, k2) — a 2×5 matrix.
// This will be composed with upstream Jacobians via the chain rule.
//
// =============================================================================
// Derivation
// =============================================================================
//
// Let D = 1 + k1·r² + k2·r⁴, where r² = u² + v².
//
// First, the partials of r² w.r.t. u and v:
//   ∂r²/∂u = 2u
//   ∂r²/∂v = 2v
//
// Partials of D:
//   ∂D/∂u = k1·2u + k2·2r²·2u = 2u·(k1 + 2k2·r²)
//   ∂D/∂v = k1·2v + k2·2r²·2v = 2v·(k1 + 2k2·r²)
//   ∂D/∂k1 = r²
//   ∂D/∂k2 = r⁴
//
// Now for px = f·D·u:
//   ∂px/∂u = f·(∂D/∂u · u + D) = f·(2u²·(k1 + 2k2·r²) + D)
//   ∂px/∂v = f·∂D/∂v · u       = f·2uv·(k1 + 2k2·r²)
//   ∂px/∂f = D·u
//   ∂px/∂k1 = f·r²·u
//   ∂px/∂k2 = f·r⁴·u
//
// And for py = f·D·v:
//   ∂py/∂u = f·∂D/∂u · v       = f·2uv·(k1 + 2k2·r²)
//   ∂py/∂v = f·(∂D/∂v · v + D) = f·(2v²·(k1 + 2k2·r²) + D)
//   ∂py/∂f = D·v
//   ∂py/∂k1 = f·r²·v
//   ∂py/∂k2 = f·r⁴·v
//
// In matrix form (2×5, columns ordered as [u, v, f, k1, k2]):
//
//   | f·(2u²·S + D)   f·2uv·S   D·u   f·r²·u   f·r⁴·u |
//   | f·2uv·S         f·(2v²·S + D)   D·v   f·r²·v   f·r⁴·v |
//
// where S = k1 + 2k2·r².
//
// =============================================================================

__device__ inline void distortWithJacobian(
    const double* uv,       // [2] normalized image coords (u, v)
    double f,               // focal length
    double k1,              // radial distortion k1
    double k2,              // radial distortion k2
    double* px_out,         // [2] final pixel coordinates (px, py)
    double* dpx_duv,        // [4] row-major 2×2: d(px,py)/d(u,v)
    double* dpx_dfk1k2      // [6] row-major 2×3: d(px,py)/d(f,k1,k2)
) {
  double u = uv[0];
  double v = uv[1];

  double r2 = u * u + v * v;
  double r4 = r2 * r2;

  double D = 1.0 + k1 * r2 + k2 * r4;

  // Output: distorted pixel coordinates
  px_out[0] = f * D * u;
  px_out[1] = f * D * v;

  // Precompute: S = k1 + 2·k2·r²
  double S = k1 + 2.0 * k2 * r2;

  // ---- d(px,py)/d(u,v) — 2×2 row-major ----
  // dpx/du = f·(2u²·S + D)
  dpx_duv[0] = f * (2.0 * u * u * S + D);
  // dpx/dv = f·2uv·S
  dpx_duv[1] = f * 2.0 * u * v * S;
  // dpy/du = f·2uv·S
  dpx_duv[2] = f * 2.0 * u * v * S;
  // dpy/dv = f·(2v²·S + D)
  dpx_duv[3] = f * (2.0 * v * v * S + D);

  // ---- d(px,py)/d(f,k1,k2) — 2×3 row-major ----
  // dpx/df = D·u
  dpx_dfk1k2[0] = D * u;
  // dpx/dk1 = f·r²·u
  dpx_dfk1k2[1] = f * r2 * u;
  // dpx/dk2 = f·r⁴·u
  dpx_dfk1k2[2] = f * r4 * u;

  // dpy/df = D·v
  dpx_dfk1k2[3] = D * v;
  // dpy/dk1 = f·r²·v
  dpx_dfk1k2[4] = f * r2 * v;
  // dpy/dk2 = f·r⁴·v
  dpx_dfk1k2[5] = f * r4 * v;
}

}  // namespace device
}  // namespace ops
}  // namespace backend
}  // namespace substral
