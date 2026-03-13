#pragma once

#include <cuda_runtime.h>

namespace substral {
namespace backend {
namespace solver {

// =============================================================================
// PCG (Preconditioned Conjugate Gradient) Solver — GPU Implementation
// =============================================================================
//
// Solves the symmetric linear system A * x = b where A is stored as a
// lower-triangle-only CSR matrix. Uses a block-diagonal Jacobi
// preconditioner (6x6 blocks, one per pose).
//
// The preconditioner inverse blocks must be precomputed via
// launchExtractAndInvertDiagBlocks() before calling this function.
//
// Algorithm (standard preconditioned CG):
//   x = 0
//   r = b                        (since A*0 = 0)
//   z = M^{-1} * r               (apply preconditioner)
//   p = z
//   rz = dot(r, z)
//
//   for k = 0..max_iter:
//     Ap = A * p                  (symmetric SpMV)
//     alpha = rz / dot(p, Ap)
//     x = x + alpha * p
//     r = r - alpha * Ap
//     if ||r|| / ||b|| < tol:    converged
//     z = M^{-1} * r             (apply preconditioner)
//     rz_new = dot(r, z)
//     beta = rz_new / rz
//     p = z + beta * p
//     rz = rz_new
//
// Returns:
//   Number of CG iterations used (positive = converged, negative = did not
//   converge within max_iterations).
//
// Parameters:
//   d_row_ptr, d_col_ind, d_values: lower-triangle CSR of the damped Hessian
//   d_b:             right-hand side vector [num_rows]
//   d_x:             solution vector [num_rows] (output, zeroed internally)
//   d_precond_blocks: precomputed 6x6 inverse blocks [num_free_poses * 36]
//   num_rows:         total dimension (num_free_poses * 6)
//   num_free_poses:   number of free poses
//   max_iterations:   maximum CG iterations
//   tolerance:        relative residual tolerance (||r|| / ||b||)
//
//   Workspace vectors (caller must allocate, each of size num_rows):
//   d_r, d_z, d_p, d_Ap
//
// =============================================================================

int solvePCG_GPU(
    const int* d_row_ptr,
    const int* d_col_ind,
    const double* d_values,
    const double* d_b,
    double* d_x,
    const double* d_precond_blocks,
    int num_rows,
    int num_free_poses,
    int max_iterations,
    double tolerance,
    // Workspace (caller-allocated, each num_rows doubles)
    double* d_r,
    double* d_z,
    double* d_p,
    double* d_Ap);

}  // namespace solver
}  // namespace backend
}  // namespace substral
