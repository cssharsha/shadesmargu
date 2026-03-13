load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

cc_library(
    name = "pipeline",
    hdrs = ["pipeline.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/backend/solver:lm_solver",
        "//viz/rerun:ba_visualizer",
    ],
)

cc_library(
    name = "ba_pipeline",
    srcs = ["ba_pipeline.cpp"],
    hdrs = ["ba_pipeline.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":pipeline",
        "//subastral/backend:backend",
        "//subastral/backend/ops:errors_cpu",
        "//subastral/backend/ops:ops_cuda",
        "//subastral/backend/solver:lm_solver",
        "//subastral/backend/solver:lm_solver_gpu",
        "//subastral/loader:bal_loader",
        "//subastral/viz:ba_visualizer",
        "//viz/rerun:ba_visualizer",
    ],
)

cc_library(
    name = "pose_graph_pipeline",
    srcs = ["pose_graph_pipeline.cpp"],
    hdrs = ["pose_graph_pipeline.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":pipeline",
        "//subastral/backend:backend",
        "//subastral/backend/solver:lm_solver",
        "//subastral/backend/solver:pose_graph_solver",
        "//subastral/loader:g2o_loader",
        "//viz/rerun:pg_visualizer",
    ],
)

cc_library(
    name = "subastral_lib",
    srcs = ["subastral.cpp"],
    hdrs = ["subastral.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":ba_pipeline",
        ":pipeline",
        ":pose_graph_pipeline",
    ],
)

cc_binary(
    name = "subastral",
    srcs = ["main.cpp"],
    linkopts = ["-lpthread"],
    deps = [
        ":subastral_lib",
    ],
)
