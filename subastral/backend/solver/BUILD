load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")

cc_library(
    name = "schur",
    srcs = ["schur.cpp"],
    hdrs = ["schur.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/backend:backend",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "lm_solver",
    srcs = ["lm_solver.cpp"],
    hdrs = [
        "lm_solver.hpp",
        "loss_function.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":schur",
        "//subastral/backend:backend",
        "//subastral/backend/ops:jacobians_cpu",
        "//subastral/backend/ops:ops_cpu",
        "@eigen//:eigen",
    ],
)

cuda_library(
    name = "gpu_solver_kernels",
    srcs = ["gpu_solver_kernels.cu"],
    hdrs = [
        "gpu_solver_kernels.cuh",
        "loss_function_gpu.cuh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/backend/lie:se3_gpu",
        "//subastral/backend/ops:ops_cuda",
    ],
)

cuda_library(
    name = "lm_solver_gpu",
    srcs = ["lm_solver_gpu.cu"],
    hdrs = ["lm_solver_gpu.cuh"],
    visibility = ["//visibility:public"],
    deps = [
        ":gpu_solver_kernels",
        ":lm_solver",
        "//subastral/backend:backend",
        "//subastral/backend/ops:jacobians_cuda",
        "@cuda_toolkit//:cusolver",
        "@eigen//:eigen",
    ],
)

cuda_library(
    name = "pose_graph_kernels",
    srcs = ["pose_graph_kernels.cu"],
    hdrs = [
        "loss_function_gpu.cuh",
        "pose_graph_kernels.cuh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/backend/lie:se3_gpu",
    ],
)

cuda_library(
    name = "pcg_solver",
    srcs = ["pcg_solver.cu"],
    hdrs = ["pcg_solver.cuh"],
    visibility = ["//visibility:public"],
    deps = [
        ":pose_graph_kernels",
    ],
)

cuda_library(
    name = "pose_graph_solver",
    srcs = ["pose_graph_solver.cu"],
    hdrs = ["pose_graph_solver.cuh"],
    visibility = ["//visibility:public"],
    deps = [
        ":lm_solver",
        ":pcg_solver",
        ":pose_graph_kernels",
        "//subastral/backend:backend",
        "@cuda_toolkit//:cusolver",
        "@cuda_toolkit//:cusparse",
    ],
)

cc_test(
    name = "loss_function_test",
    srcs = ["tests/loss_function_test.cpp"],
    deps = [
        ":lm_solver",
        "@com_google_googletest//:gtest_main",
    ],
    size = "small",
)

cc_test(
    name = "schur_test",
    srcs = ["tests/schur_test.cpp"],
    deps = [
        ":schur",
        "//subastral/backend:backend",
        "//subastral/backend/ops:jacobians_cpu",
        "@com_google_googletest//:gtest_main",
        "@eigen//:eigen",
    ],
    size = "small",
)

cc_test(
    name = "lm_solver_test",
    srcs = ["tests/lm_solver_test.cpp"],
    deps = [
        ":lm_solver",
        "//subastral/backend:backend",
        "//subastral/backend/ops:ops_cpu",
        "@com_google_googletest//:gtest_main",
        "@eigen//:eigen",
    ],
    size = "small",
)

cc_test(
    name = "pose_graph_solver_test",
    srcs = ["tests/pose_graph_solver_test.cpp"],
    deps = [
        ":pose_graph_solver",
        "//subastral/backend:backend",
        "//subastral/loader:g2o_loader",
        "@com_google_googletest//:gtest_main",
    ],
    size = "medium",
    tags = ["requires-network"],
)

cc_test(
    name = "lm_solver_gpu_test",
    srcs = ["tests/lm_solver_gpu_test.cpp"],
    deps = [
        ":gpu_solver_kernels",
        ":lm_solver_gpu",
        "//subastral/backend:backend",
        "//subastral/backend/lie:se3",
        "//subastral/backend/lie:so3",
        "//subastral/backend/ops:ops_cpu",
        "@com_google_googletest//:gtest_main",
        "@eigen//:eigen",
    ],
    size = "medium",
    tags = ["requires-network"],
)

cc_test(
    name = "pcg_solver_test",
    srcs = ["tests/pcg_solver_test.cpp"],
    deps = [
        ":pcg_solver",
        ":pose_graph_kernels",
        ":pose_graph_solver",
        "//subastral/backend:backend",
        "//subastral/loader:g2o_loader",
        "@com_google_googletest//:gtest_main",
    ],
    size = "medium",
    tags = ["requires-network"],
)
