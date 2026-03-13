load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")

# =============================================================================
# CPU libraries — Lie group infrastructure
# =============================================================================

# SO(3) Lie group operations (header-only, CPU)
cc_library(
    name = "so3",
    hdrs = ["so3.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "@eigen//:eigen",
    ],
)

# SE(3) Lie group operations (header-only, CPU, depends on SO(3))
cc_library(
    name = "se3",
    hdrs = ["se3.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":so3",
        "@eigen//:eigen",
    ],
)

# =============================================================================
# GPU libraries — Lie group infrastructure
# =============================================================================

# SO(3) GPU device functions (header-only .cuh)
cuda_library(
    name = "so3_gpu",
    hdrs = ["so3_gpu.cuh"],
    visibility = ["//visibility:public"],
)

# SE(3) GPU device functions (header-only .cuh, depends on SO(3) GPU)
cuda_library(
    name = "se3_gpu",
    hdrs = ["se3_gpu.cuh"],
    visibility = ["//visibility:public"],
    deps = [
        ":so3_gpu",
    ],
)

# GPU test kernels (compiled by nvcc)
cuda_library(
    name = "lie_gpu_test_kernels",
    srcs = ["lie_gpu_test_kernels.cu"],
    hdrs = ["lie_gpu_test_kernels.cuh"],
    visibility = ["//visibility:public"],
    deps = [
        ":se3_gpu",
    ],
)

# =============================================================================
# Tests
# =============================================================================

cc_test(
    name = "so3_test",
    srcs = ["tests/so3_test.cpp"],
    deps = [
        ":so3",
        "@com_google_googletest//:gtest_main",
        "@eigen//:eigen",
    ],
    size = "small",
)

cc_test(
    name = "se3_test",
    srcs = ["tests/se3_test.cpp"],
    deps = [
        ":se3",
        "@com_google_googletest//:gtest_main",
        "@eigen//:eigen",
    ],
    size = "small",
)

# GPU vs CPU consistency test
cc_test(
    name = "lie_gpu_test",
    srcs = ["tests/lie_gpu_test.cpp"],
    deps = [
        ":lie_gpu_test_kernels",
        ":se3",
        ":so3",
        "@com_google_googletest//:gtest_main",
        "@eigen//:eigen",
    ],
    size = "small",
    tags = ["requires-network"],
)
