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
        "//viz/opencv:ba_visualizer",
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
    name = "chordal_init_pipeline",
    srcs = ["chordal_init_pipeline.cpp"],
    hdrs = ["chordal_init_pipeline.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":pipeline",
        "//subastral/backend:backend",
        "//subastral/backend/solver:lm_solver",
        "//subastral/backend/solver:pose_graph_solver",
        "//subastral/loader:g2o_loader",
        "//viz/rerun:pg_visualizer",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "visual_frontend_pipeline",
    srcs = ["visual_frontend_pipeline.cpp"],
    hdrs = ["visual_frontend_pipeline.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":pipeline",
        "//subastral/frontend:depth_initializer",
        "//subastral/frontend:feature_extractor",
        "//subastral/frontend:feature_extractor_cpu",
        "//subastral/frontend:feature_matcher",
        "//subastral/frontend:feature_matcher_cpu",
        "//subastral/frontend:imu_initializer",
        "//subastral/frontend:imu_preintegrator",
        "//subastral/frontend:two_view_geometry",
        "//subastral/loader:euroc_loader",
        "//subastral/loader:tum_loader",
        "//subastral/types:types",
        "//viz/rerun:vfe_visualizer",
        "@opencv//:opencv",
    ],
)

cc_library(
    name = "subastral_lib",
    srcs = ["subastral.cpp"],
    hdrs = ["subastral.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":ba_pipeline",
        ":chordal_init_pipeline",
        ":pipeline",
        ":pose_graph_pipeline",
        ":visual_frontend_pipeline",
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
