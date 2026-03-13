load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "kalman",
    srcs = [
        "kalman/ekf.cpp",
        "kalman/models.cpp",
    ],
    hdrs = [
        "kalman/ekf.hpp",
        "kalman/models.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = ["@eigen//:eigen"],
)

cc_binary(
    name = "run_ekf",
    srcs = ["kalman/run_ekf.cpp"],
    data = [
        "ekf/inputs.txt",
        "ekf/sensor_readings.txt",
    ],
    deps = [
        ":kalman",
        "@eigen//:eigen",
        "@matplotlib_cpp//:matplotlib_cpp",
    ],
    copts = ["-std=c++17"],
)
