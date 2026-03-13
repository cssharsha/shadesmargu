load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "ba_visualizer",
    srcs = ["ba_visualizer.cpp"],
    hdrs = ["ba_visualizer.hpp"],
    deps = [
        "@rerun_cpp_sdk//:rerun_sdk",
        "//subastral/backend:backend",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "pg_visualizer",
    srcs = ["pg_visualizer.cpp"],
    hdrs = ["pg_visualizer.hpp"],
    deps = [
        ":ba_visualizer",
        "@rerun_cpp_sdk//:rerun_sdk",
        "//subastral/backend:backend",
        "//subastral/backend/lie:se3",
    ],
    visibility = ["//visibility:public"],
)
