#pragma once

#include <cuda_runtime.h>

#include <cmath>

namespace substral {
namespace backend {
namespace ops {
namespace device {

// =============================================================================
// Rodrigues Rotation with Analytical Jacobian
// =============================================================================
//
// The Rodrigues formula rotates a point P by an angle-axis vector ω:
//
//   P' = R(ω) · P
//
// where R(ω) = I·cosθ + (1 - cosθ)·k·kᵀ + sinθ·[k]×
//       θ = ‖ω‖,  k = ω / θ  (unit axis)
//
// Expanding (Rodrigues formula in vector form):
//   P' = P·cosθ + (k × P)·sinθ + k·(k·P)·(1 - cosθ)
//
// We need two Jacobians:
//   dP'/dω  (3×3)  — how the rotated point changes w.r.t. the angle-axis
//   dP'/dP  (3×3)  — how the rotated point changes w.r.t. the input point
//                     (this is simply the rotation matrix R itself)
//
// =============================================================================
// Derivation of dP'/dω
// =============================================================================
//
// We use the result from Gallego & Yezzi (2015) / Ceres Solver:
//
// For the general case (θ > ε):
//
//   dP'/dω = d(R·P)/dω
//
// Let's derive column j of dP'/dω, i.e. ∂P'/∂ω_j.
//
// Starting from:
//   P' = cosθ · P  +  sinθ · (k × P)  +  (1-cosθ) · (k·P) · k
//
// We need the partials of θ, k, and the trig functions w.r.t. ω_j:
//
//   ∂θ/∂ω_j = ω_j / θ = k_j
//
//   ∂k_i/∂ω_j = (δ_ij - k_i·k_j) / θ
//     where δ_ij is the Kronecker delta
//
// Now differentiate each of the three terms:
//
// Term 1:  d(cosθ · P)/dω_j = -sinθ · k_j · P
//
// Term 2:  d(sinθ · (k × P))/dω_j
//        = cosθ · k_j · (k × P)  +  sinθ · (∂k/∂ω_j × P)
//
//   where ∂k/∂ω_j = (e_j - k_j·k) / θ,  so:
//        (∂k/∂ω_j × P) = ((e_j - k_j·k) × P) / θ
//                       = (e_j × P - k_j · (k × P)) / θ
//
//   Substituting:
//        = cosθ · k_j · (k × P) + (sinθ/θ) · (e_j × P - k_j · (k × P))
//        = cosθ · k_j · (k × P) + (sinθ/θ) · (e_j × P) - (sinθ/θ)·k_j·(k×P)
//        = (cosθ - sinθ/θ)·k_j·(k × P) + (sinθ/θ)·(e_j × P)
//
// Term 3:  d((1-cosθ)·(k·P)·k)/dω_j
//        = sinθ·k_j·(k·P)·k
//          + (1-cosθ)·[∂(k·P)/∂ω_j · k  +  (k·P) · ∂k/∂ω_j]
//
//   ∂(k·P)/∂ω_j = (∂k/∂ω_j)·P = (P_j - k_j·(k·P)) / θ
//
//   So Term 3 =
//     sinθ·k_j·(k·P)·k
//     + ((1-cosθ)/θ)·(P_j - k_j·(k·P))·k
//     + ((1-cosθ)/θ)·(k·P)·(e_j - k_j·k)
//
//   = sinθ·k_j·(k·P)·k
//     + ((1-cosθ)/θ)·P_j·k
//     - ((1-cosθ)/θ)·k_j·(k·P)·k
//     + ((1-cosθ)/θ)·(k·P)·e_j
//     - ((1-cosθ)/θ)·(k·P)·k_j·k
//
//   = [sinθ - 2(1-cosθ)/θ]·k_j·(k·P)·k
//     + ((1-cosθ)/θ)·[P_j·k + (k·P)·e_j]
//
// Combining all three terms and collecting by coefficient:
//
//   ∂P'/∂ω_j = -sinθ·k_j·P
//              + (cosθ - sinθ/θ)·k_j·(k × P)
//              + (sinθ/θ)·(e_j × P)
//              + [sinθ - 2(1-cosθ)/θ]·k_j·(k·P)·k
//              + ((1-cosθ)/θ)·[P_j·k + (k·P)·e_j]
//
// This can be written in matrix form. Define the helper scalars:
//   a = sinθ/θ                    (→ 1 as θ→0)
//   b = (1 - cosθ)/θ²            (→ 0.5 as θ→0)
//   c = (1 - a)/θ²               (→ 1/6 as θ→0)
//     = (1 - sinθ/θ)/θ²
//     = (θ - sinθ)/θ³
//
// Then the full Jacobian can be expressed as:
//
//   dP'/dω = -[P']× · (sinθ·I + (1-cosθ)·[k]×) / θ
//
// ...but the cleanest implementation avoids recomputing R and instead uses:
//
//   dP'/dω = [P']× · ( -a·I  -  b·[ω]×  -  c·ω·ωᵀ )    ... (*)
//          + (k·P)·[ ... ] ...
//
// Actually, the simplest correct formulation (used by Ceres) is:
//
//   ∂(R·P)/∂ω = -[R·P]× · J_l(ω)
//
// where J_l(ω) is the left Jacobian of SO(3). But for implementation clarity
// we'll use the direct column-by-column approach derived above.
//
// For implementation, we use the equivalent compact form:
//
//   dP'/dω_j = (sinθ/θ)·(e_j × P)
//            + ((1-cosθ)/θ²)·(ω_j·(ω × P) + ω × (e_j × P))
//            + (1/θ²)·(1 - sinθ/θ)·(ω_j·(ω·P)·ω - ω_j·θ²·P
//              ... )
//
// For clarity and numerical stability, we implement column-by-column using
// precomputed intermediates.
//
// =============================================================================
// Small angle case (θ < ε):
// =============================================================================
//
// When θ → 0:
//   R ≈ I + [ω]×
//   P' ≈ P + ω × P
//
//   dP'/dω ≈ -[P]×   (the skew-symmetric matrix of P)
//
// This is because d(ω × P)/dω = -[P]×.
//
// dP'/dP ≈ I + [ω]× ≈ I  (to first order, but we keep [ω]× for accuracy)
//
// =============================================================================

__device__ inline void rotatePointWithJacobian(
    const double* w,       // [3] angle-axis vector ω
    const double* P,       // [3] input point
    double* P_out,         // [3] rotated point P' = R(ω)·P
    double* dPout_dw,      // [9] row-major 3×3: dP'/dω
    double* dPout_dP       // [9] row-major 3×3: dP'/dP = R(ω)
) {
  double theta_sq = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
  double theta = sqrt(theta_sq);

  // ---- Small angle case ----
  if (theta < 1e-10) {
    // P' = P + ω × P
    // (same as your existing rotatePoint small-angle branch)
    P_out[0] = P[0] + (w[1] * P[2] - w[2] * P[1]);
    P_out[1] = P[1] + (w[2] * P[0] - w[0] * P[2]);
    P_out[2] = P[2] + (w[0] * P[1] - w[1] * P[0]);

    // dP'/dω = -[P]×
    // -[P]× = | 0     P[2]  -P[1] |
    //         | -P[2]  0     P[0]  |
    //         | P[1]  -P[0]  0     |
    dPout_dw[0] = 0.0;     dPout_dw[1] = P[2];    dPout_dw[2] = -P[1];
    dPout_dw[3] = -P[2];   dPout_dw[4] = 0.0;     dPout_dw[5] = P[0];
    dPout_dw[6] = P[1];    dPout_dw[7] = -P[0];   dPout_dw[8] = 0.0;

    // dP'/dP = I + [ω]×
    // [ω]× = | 0     -w[2]   w[1] |
    //        | w[2]   0     -w[0] |
    //        | -w[1]  w[0]   0    |
    dPout_dP[0] = 1.0;     dPout_dP[1] = -w[2];   dPout_dP[2] = w[1];
    dPout_dP[3] = w[2];    dPout_dP[4] = 1.0;     dPout_dP[5] = -w[0];
    dPout_dP[6] = -w[1];   dPout_dP[7] = w[0];    dPout_dP[8] = 1.0;

    return;
  }

  // ---- General case ----
  double k[3] = {w[0] / theta, w[1] / theta, w[2] / theta};
  double cos_theta = cos(theta);
  double sin_theta = sin(theta);
  double one_minus_cos = 1.0 - cos_theta;

  // Precompute intermediates
  double k_dot_p = k[0] * P[0] + k[1] * P[1] + k[2] * P[2];

  double k_cross_p[3];
  k_cross_p[0] = k[1] * P[2] - k[2] * P[1];
  k_cross_p[1] = k[2] * P[0] - k[0] * P[2];
  k_cross_p[2] = k[0] * P[1] - k[1] * P[0];

  // Compute rotated point (Rodrigues formula)
  P_out[0] = P[0] * cos_theta + k_cross_p[0] * sin_theta +
             k[0] * k_dot_p * one_minus_cos;
  P_out[1] = P[1] * cos_theta + k_cross_p[1] * sin_theta +
             k[1] * k_dot_p * one_minus_cos;
  P_out[2] = P[2] * cos_theta + k_cross_p[2] * sin_theta +
             k[2] * k_dot_p * one_minus_cos;

  // ---- Compute dP'/dω (3×3) ----
  //
  // Helper scalars (numerically stable):
  //   a = sinθ / θ
  //   b = (1 - cosθ) / θ²
  double a = sin_theta / theta;
  double b = one_minus_cos / theta_sq;

  // The simplest correct derivation:
  //
  // R(ω) = I + a·[ω]× + b·[ω]×²
  //   where a = sinθ/θ, b = (1-cosθ)/θ²
  //
  // d(R·P)/dω_j = ∂a/∂ω_j · [ω]×·P + a · ∂([ω]×·P)/∂ω_j
  //             + ∂b/∂ω_j · [ω]×²·P + b · ∂([ω]×²·P)/∂ω_j
  //
  // where:
  //   ∂a/∂ω_j = (θ·cosθ - sinθ) / θ² · k_j = (cosθ/θ - sinθ/θ²) · ω_j/θ · θ
  //           = (cosθ - a) · k_j / θ = (cosθ - a) · ω_j / θ²
  //
  //   ∂b/∂ω_j = (sinθ·θ² - 2θ(1-cosθ)) / θ⁴ · ω_j
  //           = (sinθ/θ - 2(1-cosθ)/θ²) / θ · ω_j ... simplify:
  //           = (a - 2b) · ω_j / θ²
  //
  //   ∂([ω]×·P)/∂ω_j = [e_j]× · P = e_j × P
  //
  //   ∂([ω]×²·P)/∂ω_j = [e_j]×·[ω]×·P + [ω]×·[e_j]×·P
  //                    = e_j × (ω × P) + ω × (e_j × P)
  //
  // So:
  //   dP'/dω_j = (cosθ - a)/θ² · ω_j · (ω × P)
  //            + a · (e_j × P)
  //            + (a - 2b)/θ² · ω_j · (ω×(ω×P))
  //            + b · [e_j × (ω × P) + ω × (e_j × P)]
  //
  // Let's define:
  //   wxP = ω × P                   (already have as θ·k_cross_p)
  //   wwxP = ω × (ω × P)           (= [ω]×² · P)
  //   da = (cosθ - a) / θ²
  //   db = (a - 2b) / θ²

  double wxP[3];  // ω × P (not k × P)
  wxP[0] = w[1] * P[2] - w[2] * P[1];
  wxP[1] = w[2] * P[0] - w[0] * P[2];
  wxP[2] = w[0] * P[1] - w[1] * P[0];

  double wwxP[3];  // ω × (ω × P) = [ω]×² · P
  wwxP[0] = w[1] * wxP[2] - w[2] * wxP[1];
  wwxP[1] = w[2] * wxP[0] - w[0] * wxP[2];
  wwxP[2] = w[0] * wxP[1] - w[1] * wxP[0];

  double da = (cos_theta - a) / theta_sq;
  double db = (a - 2.0 * b) / theta_sq;

  // Column j=0: e_0 × P = (0, P[2], -P[1])
  //             e_0 × (ω×P) = (0, wxP[2], -wxP[1])
  //             ω × (e_0 × P) = (w[1]*(-P[1]) - w[2]*P[2],
  //                               w[2]*0 - w[0]*(-P[1]),
  //                               w[0]*P[2] - w[1]*0)
  //                            = (-w[1]*P[1] - w[2]*P[2], w[0]*P[1], w[0]*P[2])
  // But it's cleaner to just compute e_j × P and the cross products inline.

  // For each column j, compute:
  //   col_j = da * w[j] * wxP
  //         + a * (e_j × P)
  //         + db * w[j] * wwxP
  //         + b * (e_j × wxP + w × (e_j × P))

  // Helper: e_j × v  for j=0,1,2
  // (a × b)_i = a_{i+1}*b_{i+2} - a_{i+2}*b_{i+1}  (indices mod 3)
  //
  // e_0 × v = (0·v2 - 0·v1, 0·v0 - 1·v2, 1·v1 - 0·v0) = (0, -v[2], v[1])
  // e_1 × v = (1·v2 - 0·v1, 0·v0 - 0·v2, 0·v1 - 1·v0) = (v[2], 0, -v[0])
  // e_2 × v = (0·v2 - 1·v1, 1·v0 - 0·v2, 0·v1 - 0·v0) = (-v[1], v[0], 0)

  // Column 0 (∂P'/∂ω_0)
  {
    // e_0 × P = (0, -P[2], P[1])
    double ejxP[3] = {0.0, -P[2], P[1]};
    // e_0 × wxP = (0, -wxP[2], wxP[1])
    double ejxwxP[3] = {0.0, -wxP[2], wxP[1]};
    // ω × (e_0 × P) = ω × (0, -P[2], P[1])
    double wxejxP[3] = {w[1] * P[1] - w[2] * (-P[2]),
                        w[2] * 0.0 - w[0] * P[1],
                        w[0] * (-P[2]) - w[1] * 0.0};

    for (int i = 0; i < 3; ++i) {
      dPout_dw[i * 3 + 0] = da * w[0] * wxP[i] + a * ejxP[i] +
                             db * w[0] * wwxP[i] +
                             b * (ejxwxP[i] + wxejxP[i]);
    }
  }

  // Column 1 (∂P'/∂ω_1)
  {
    // e_1 × P = (P[2], 0, -P[0])
    double ejxP[3] = {P[2], 0.0, -P[0]};
    // e_1 × wxP = (wxP[2], 0, -wxP[0])
    double ejxwxP[3] = {wxP[2], 0.0, -wxP[0]};
    // ω × (e_1 × P) = ω × (P[2], 0, -P[0])
    double wxejxP[3] = {w[1] * (-P[0]) - w[2] * 0.0,
                        w[2] * P[2] - w[0] * (-P[0]),
                        w[0] * 0.0 - w[1] * P[2]};

    for (int i = 0; i < 3; ++i) {
      dPout_dw[i * 3 + 1] = da * w[1] * wxP[i] + a * ejxP[i] +
                             db * w[1] * wwxP[i] +
                             b * (ejxwxP[i] + wxejxP[i]);
    }
  }

  // Column 2 (∂P'/∂ω_2)
  {
    // e_2 × P = (-P[1], P[0], 0)
    double ejxP[3] = {-P[1], P[0], 0.0};
    // e_2 × wxP = (-wxP[1], wxP[0], 0)
    double ejxwxP[3] = {-wxP[1], wxP[0], 0.0};
    // ω × (e_2 × P) = ω × (-P[1], P[0], 0)
    double wxejxP[3] = {w[1] * 0.0 - w[2] * P[0],
                        w[2] * (-P[1]) - w[0] * 0.0,
                        w[0] * P[0] - w[1] * (-P[1])};

    for (int i = 0; i < 3; ++i) {
      dPout_dw[i * 3 + 2] = da * w[2] * wxP[i] + a * ejxP[i] +
                             db * w[2] * wwxP[i] +
                             b * (ejxwxP[i] + wxejxP[i]);
    }
  }

  // ---- Compute dP'/dP (3×3) = R(ω) ----
  //
  // R = I + a·[ω]× + b·[ω]×²
  //
  // [ω]× = |  0    -w[2]   w[1] |
  //        |  w[2]   0    -w[0] |
  //        | -w[1]  w[0]   0    |
  //
  // [ω]×² = | -(w[1]²+w[2]²)   w[0]w[1]         w[0]w[2]       |
  //         |  w[0]w[1]        -(w[0]²+w[2]²)    w[1]w[2]       |
  //         |  w[0]w[2]         w[1]w[2]        -(w[0]²+w[1]²)  |

  // Row 0
  dPout_dP[0] = 1.0 + b * (-(w[1] * w[1] + w[2] * w[2]));
  dPout_dP[1] = a * (-w[2]) + b * (w[0] * w[1]);
  dPout_dP[2] = a * w[1] + b * (w[0] * w[2]);

  // Row 1
  dPout_dP[3] = a * w[2] + b * (w[0] * w[1]);
  dPout_dP[4] = 1.0 + b * (-(w[0] * w[0] + w[2] * w[2]));
  dPout_dP[5] = a * (-w[0]) + b * (w[1] * w[2]);

  // Row 2
  dPout_dP[6] = a * (-w[1]) + b * (w[0] * w[2]);
  dPout_dP[7] = a * w[0] + b * (w[1] * w[2]);
  dPout_dP[8] = 1.0 + b * (-(w[0] * w[0] + w[1] * w[1]));
}

}  // namespace device
}  // namespace ops
}  // namespace backend
}  // namespace substral
