load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "ba_visualizer",
    srcs = ["ba_visualizer.cpp"],
    hdrs = ["ba_visualizer.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/backend:backend",
        "@opencv//:opencv",
    ],
)
