#pragma once

#include <cuda_runtime.h>

namespace substral {
namespace backend {
namespace lie {

// Kernel declarations for GPU Lie group tests.
// Each kernel tests one operation on a batch of inputs.

// Test SO(3) exp: w[N*3] -> R[N*9]
void launchExpSO3Test(const double* d_w, double* d_R, int N);

// Test SO(3) log: R[N*9] -> w[N*3]
void launchLogSO3Test(const double* d_R, double* d_w, int N);

// Test SO(3) left Jacobian: w[N*3] -> Jl[N*9]
void launchLeftJacobianSO3Test(const double* d_w, double* d_Jl, int N);

// Test SO(3) left Jacobian inverse: w[N*3] -> Jl_inv[N*9]
void launchLeftJacobianInvSO3Test(const double* d_w, double* d_Jl_inv, int N);

// Test SE(3) exp: xi[N*6] -> R[N*9], t[N*3]
void launchExpSE3Test(const double* d_xi, double* d_R, double* d_t, int N);

// Test SE(3) log: R[N*9], t[N*3] -> xi[N*6]
void launchLogSE3Test(const double* d_R, const double* d_t, double* d_xi,
                       int N);

// Test SE(3) transform point: R[N*9], t[N*3], p[N*3] -> p_out[N*3]
void launchTransformPointTest(const double* d_R, const double* d_t,
                               const double* d_p, double* d_p_out, int N);

}  // namespace lie
}  // namespace backend
}  // namespace substral
