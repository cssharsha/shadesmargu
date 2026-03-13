#include "subastral/backend/lie/lie_gpu_test_kernels.cuh"
#include "subastral/backend/lie/se3_gpu.cuh"

namespace substral {
namespace backend {
namespace lie {

// =============================================================================
// Kernel implementations
// =============================================================================

__global__ void expSO3Kernel(const double* w, double* R, int N) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= N) return;
  device::expSO3(w + idx * 3, R + idx * 9);
}

__global__ void logSO3Kernel(const double* R, double* w, int N) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= N) return;
  device::logSO3(R + idx * 9, w + idx * 3);
}

__global__ void leftJacobianSO3Kernel(const double* w, double* Jl, int N) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= N) return;
  device::leftJacobianSO3(w + idx * 3, Jl + idx * 9);
}

__global__ void leftJacobianInvSO3Kernel(const double* w, double* Jl_inv,
                                          int N) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= N) return;
  device::leftJacobianInverseSO3(w + idx * 3, Jl_inv + idx * 9);
}

__global__ void expSE3Kernel(const double* xi, double* R, double* t, int N) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= N) return;
  device::expSE3(xi + idx * 6, R + idx * 9, t + idx * 3);
}

__global__ void logSE3Kernel(const double* R, const double* t, double* xi,
                              int N) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= N) return;
  device::logSE3(R + idx * 9, t + idx * 3, xi + idx * 6);
}

__global__ void transformPointKernel(const double* R, const double* t,
                                      const double* p, double* p_out, int N) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= N) return;
  device::transformPoint(R + idx * 9, t + idx * 3, p + idx * 3,
                          p_out + idx * 3);
}

// =============================================================================
// Launch wrappers
// =============================================================================

void launchExpSO3Test(const double* d_w, double* d_R, int N) {
  int threads = 256;
  int blocks = (N + threads - 1) / threads;
  expSO3Kernel<<<blocks, threads>>>(d_w, d_R, N);
}

void launchLogSO3Test(const double* d_R, double* d_w, int N) {
  int threads = 256;
  int blocks = (N + threads - 1) / threads;
  logSO3Kernel<<<blocks, threads>>>(d_R, d_w, N);
}

void launchLeftJacobianSO3Test(const double* d_w, double* d_Jl, int N) {
  int threads = 256;
  int blocks = (N + threads - 1) / threads;
  leftJacobianSO3Kernel<<<blocks, threads>>>(d_w, d_Jl, N);
}

void launchLeftJacobianInvSO3Test(const double* d_w, double* d_Jl_inv,
                                   int N) {
  int threads = 256;
  int blocks = (N + threads - 1) / threads;
  leftJacobianInvSO3Kernel<<<blocks, threads>>>(d_w, d_Jl_inv, N);
}

void launchExpSE3Test(const double* d_xi, double* d_R, double* d_t, int N) {
  int threads = 256;
  int blocks = (N + threads - 1) / threads;
  expSE3Kernel<<<blocks, threads>>>(d_xi, d_R, d_t, N);
}

void launchLogSE3Test(const double* d_R, const double* d_t, double* d_xi,
                       int N) {
  int threads = 256;
  int blocks = (N + threads - 1) / threads;
  logSE3Kernel<<<blocks, threads>>>(d_R, d_t, d_xi, N);
}

void launchTransformPointTest(const double* d_R, const double* d_t,
                               const double* d_p, double* d_p_out, int N) {
  int threads = 256;
  int blocks = (N + threads - 1) / threads;
  transformPointKernel<<<blocks, threads>>>(d_R, d_t, d_p, d_p_out, N);
}

}  // namespace lie
}  // namespace backend
}  // namespace substral
