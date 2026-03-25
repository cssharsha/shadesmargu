load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

cc_library(
    name = "bal_loader",
    srcs = ["bal_loader.cpp"],
    hdrs = ["bal_loader.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/backend:backend",
    ],
)

cc_library(
    name = "g2o_loader",
    srcs = ["g2o_loader.cpp"],
    hdrs = ["g2o_loader.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/backend:backend",
    ],
)

cc_test(
    name = "g2o_loader_test",
    srcs = ["g2o_loader_test.cpp"],
    deps = [
        ":g2o_loader",
        "//subastral/backend:backend",
        "@com_google_googletest//:gtest_main",
    ],
    size = "small",
)

cc_library(
    name = "tum_loader",
    srcs = ["tum_loader.cpp"],
    hdrs = ["tum_loader.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/types:types",
    ],
)

cc_test(
    name = "tum_loader_test",
    srcs = ["tum_loader_test.cpp"],
    deps = [
        ":tum_loader",
        "@com_google_googletest//:gtest_main",
    ],
    size = "small",
)

cc_binary(
    name = "load_bal",
    srcs = ["bal_loader_main.cpp"],
    deps = [
        ":bal_loader",
        "//subastral/backend:backend",
    ],
)
