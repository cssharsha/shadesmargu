#include "subastral/backend/solver/pcg_solver.cuh"

#include <thrust/device_ptr.h>
#include <thrust/inner_product.h>

#include <cmath>

#include "subastral/backend/solver/pose_graph_kernels.cuh"

namespace substral {
namespace backend {
namespace solver {

// =============================================================================
// PCG Solver — GPU Implementation
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
    double* d_r,
    double* d_z,
    double* d_p,
    double* d_Ap) {

  // Wrap raw pointers for thrust reductions
  thrust::device_ptr<double> t_r(d_r);
  thrust::device_ptr<double> t_z(d_z);
  thrust::device_ptr<double> t_p(d_p);
  thrust::device_ptr<double> t_Ap(d_Ap);
  thrust::device_ptr<double> t_x(d_x);
  thrust::device_ptr<const double> t_b(d_b);

  // x = 0
  cudaMemset(d_x, 0, num_rows * sizeof(double));

  // r = b (since A*0 = 0, residual = b - A*x = b)
  cudaMemcpy(d_r, d_b, num_rows * sizeof(double), cudaMemcpyDeviceToDevice);

  // Compute ||b|| for relative convergence check
  double b_norm_sq = thrust::inner_product(t_b, t_b + num_rows, t_b, 0.0);
  double b_norm = std::sqrt(b_norm_sq);

  if (b_norm < 1e-30) {
    // RHS is essentially zero — x = 0 is the solution
    return 0;
  }

  double tol_abs = tolerance * b_norm;

  // z = M^{-1} * r
  pg_gpu::launchApplyBlockPreconditioner(d_precond_blocks, d_r,
                                          num_free_poses, d_z);
  cudaDeviceSynchronize();

  // p = z
  cudaMemcpy(d_p, d_z, num_rows * sizeof(double), cudaMemcpyDeviceToDevice);

  // rz = dot(r, z)
  double rz = thrust::inner_product(t_r, t_r + num_rows, t_z, 0.0);

  for (int k = 0; k < max_iterations; ++k) {
    // Ap = A * p  (symmetric SpMV, y must be zeroed first)
    cudaMemset(d_Ap, 0, num_rows * sizeof(double));
    pg_gpu::launchSymmetricSpMV(d_row_ptr, d_col_ind, d_values,
                                 d_p, d_Ap, num_rows);
    cudaDeviceSynchronize();

    // pAp = dot(p, Ap)
    double pAp = thrust::inner_product(t_p, t_p + num_rows, t_Ap, 0.0);

    if (std::abs(pAp) < 1e-30) {
      // Direction is in the null space — can't make progress
      return -(k + 1);
    }

    double alpha = rz / pAp;

    // x = x + alpha * p
    pg_gpu::launchAxpy(alpha, d_p, d_x, num_rows);

    // r = r - alpha * Ap
    pg_gpu::launchAxpy(-alpha, d_Ap, d_r, num_rows);
    cudaDeviceSynchronize();

    // Check convergence: ||r|| < tol * ||b||
    double r_norm_sq = thrust::inner_product(t_r, t_r + num_rows, t_r, 0.0);
    double r_norm = std::sqrt(r_norm_sq);

    if (r_norm < tol_abs) {
      return k + 1;  // Converged (positive = success)
    }

    // z = M^{-1} * r
    pg_gpu::launchApplyBlockPreconditioner(d_precond_blocks, d_r,
                                            num_free_poses, d_z);
    cudaDeviceSynchronize();

    // rz_new = dot(r, z)
    double rz_new = thrust::inner_product(t_r, t_r + num_rows, t_z, 0.0);

    if (std::abs(rz) < 1e-30) {
      return -(k + 1);
    }

    double beta = rz_new / rz;

    // p = z + beta * p
    pg_gpu::launchXpby(d_z, beta, d_p, num_rows);
    cudaDeviceSynchronize();

    rz = rz_new;
  }

  return -max_iterations;  // Did not converge
}

}  // namespace solver
}  // namespace backend
}  // namespace substral
