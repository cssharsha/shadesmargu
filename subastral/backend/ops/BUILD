load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")

# =============================================================================
# CPU libraries — compiled by clang, no CUDA dependencies
# =============================================================================

# CPU projection (project_cpu function)
cc_library(
    name = "ops_cpu",
    srcs = ["projection_cpu.cpp"],
    hdrs = ["projection_cpu.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/backend:backend",
        "@eigen//:eigen",
    ],
)

# CPU Jacobians (projectWithJacobianCPU and sub-stage functions)
cc_library(
    name = "jacobians_cpu",
    srcs = ["project_with_jacobian_cpu.cpp"],
    hdrs = ["project_with_jacobian_cpu.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":ops_cpu",
        "@eigen//:eigen",
    ],
)

# CPU Lie group Jacobians (projectWithJacobianLieCPU)
cc_library(
    name = "jacobians_lie_cpu",
    srcs = ["project_with_jacobian_lie_cpu.cpp"],
    hdrs = ["project_with_jacobian_lie_cpu.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":jacobians_cpu",
        "//subastral/backend/lie:so3",
        "@eigen//:eigen",
    ],
)

# CPU error computation (computeProjectionErrorCPU)
cc_library(
    name = "errors_cpu",
    srcs = ["errors_cpu.cpp"],
    hdrs = ["errors.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":ops_cpu",
        "//subastral/backend:backend",
    ],
)

# CPU batch residual + Jacobian computation (computeResidualsAndJacobiansCPU)
cc_library(
    name = "jacobians_batch_cpu",
    srcs = ["jacobians_cpu.cpp"],
    hdrs = ["jacobians.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":jacobians_cpu",
        "//subastral/backend:backend",
    ],
)

# =============================================================================
# GPU libraries — compiled by nvcc, no CPU projection code
# =============================================================================

# GPU error computation (kernel + computeProjectionErrorGPU)
cuda_library(
    name = "ops_cuda",
    srcs = ["errors.cu"],
    hdrs = ["errors.cuh", "projection_gpu.cuh"],
    visibility = ["//visibility:public"],
    deps = [
        ":errors_cpu",
        "//subastral/backend:backend",
    ],
)

# GPU Jacobian library (fused residual + Jacobian kernel)
cuda_library(
    name = "jacobians_cuda",
    srcs = ["jacobians.cu"],
    hdrs = [
        "jacobians.cuh",
        "distortion_gpu.cuh",
        "perspective_gpu.cuh",
        "rodrigues_gpu.cuh",
        "project_with_jacobian_gpu.cuh",
        "project_with_jacobian_lie_gpu.cuh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":jacobians_batch_cpu",
        "//subastral/backend:backend",
    ],
)
