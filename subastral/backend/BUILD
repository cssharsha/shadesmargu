load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")

cc_library(
    name = "backend",
    hdrs = ["common.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/backend/graph:ba_types",
        "//subastral/backend/graph:landmark_types",
        "//subastral/backend/graph:pose_graph_types",
    ],
)

cc_test(
    name = "common_test",
    srcs = ["tests/common_test.cpp"],
    deps = [
        ":backend",
        "@com_google_googletest//:gtest_main",
    ],
    size = "small",
)

# CPU-only Jacobian test: finite-difference validation
# This test does NOT require a GPU.
cc_test(
    name = "jacobian_test",
    srcs = ["tests/jacobian_test.cpp"],
    deps = [
        "//subastral/backend/ops:jacobians_cpu",
        "//subastral/backend/ops:ops_cpu",
        "@com_google_googletest//:gtest_main",
        "@eigen//:eigen",
    ],
    size = "small",
)

# GPU vs CPU consistency test (non-Lie and Lie Jacobians)
# Requires a GPU to run.
cc_test(
    name = "jacobian_gpu_test",
    srcs = ["tests/jacobian_gpu_test.cpp"],
    deps = [
        ":backend",
        "//subastral/backend/ops:jacobians_batch_cpu",
        "//subastral/backend/ops:jacobians_cuda",
        "//subastral/backend/ops:jacobians_lie_cpu",
        "@com_google_googletest//:gtest_main",
    ],
    size = "small",
    tags = ["requires-network"],  # GPU test, may need special scheduling
)

# CPU-only Lie group Jacobian test: finite-difference validation
cc_test(
    name = "jacobian_lie_test",
    srcs = ["tests/jacobian_lie_test.cpp"],
    deps = [
        "//subastral/backend/lie:se3",
        "//subastral/backend/lie:so3",
        "//subastral/backend/ops:jacobians_lie_cpu",
        "//subastral/backend/ops:ops_cpu",
        "@com_google_googletest//:gtest_main",
        "@eigen//:eigen",
    ],
    size = "small",
)

