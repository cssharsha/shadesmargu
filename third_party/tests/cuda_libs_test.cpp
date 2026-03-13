#include <cublas_v2.h>
#include <cusolverDn.h>
#include <thrust/version.h>
// #include <cub/cub.cuh> // Avoiding device code compilation issues in
// host-only cc_test
#include <cublas_v2.h>
#include <cusolverDn.h>

#include <iostream>

int main() {
  std::cout << "Thrust v" << THRUST_MAJOR_VERSION << "." << THRUST_MINOR_VERSION
            << std::endl;

  // Just instantiate something to check linking
  // cublasHandle_t handle;
  // cusolverDnHandle_t solver_handle;

  return 0;
}
