load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "types",
    hdrs = [
        "camera.h",
        "frame.h",
        "map.h",
        "map_point.h",
        "sensor_config.h",
        "transform_tree.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@eigen//:eigen",
        "@opencv//:opencv",
    ],
)

cc_test(
    name = "transform_tree_test",
    srcs = ["transform_tree_test.cpp"],
    deps = [
        ":types",
        "@eigen//:eigen",
        "@com_google_googletest//:gtest_main",
    ],
    size = "small",
)
