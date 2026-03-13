load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "fit",
    hdrs = [
        "hough_line.hpp",
        "model.hpp",
        "ransac_line.hpp",
        "ransac_circle.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = ["@eigen//:eigen"],
)

cc_test(
    name = "ransac_circle_test",
    srcs = ["ransac_circle_test.cpp"],
    deps = [
        ":fit",
        "@eigen//:eigen",
    ],
    size = "small",
)

cc_test(
    name = "fit_test",
    srcs = ["fit_test.cpp"],
    deps = [
        ":fit",
        "@eigen//:eigen",
    ],
    size = "small",
)
