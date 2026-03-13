load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "eigen_test",
    srcs = ["eigen_test.cpp"],
    deps = ["@eigen//:eigen"],
    size = "small",
)

cc_test(
    name = "opencv_test",
    srcs = ["opencv_test.cpp"],
    deps = ["@opencv//:opencv"],
    size = "small",
)

cc_test(
    name = "cuda_libs_test",
    srcs = ["cuda_libs_test.cpp"],
    deps = [
        "@cuda_toolkit//:thrust",
        "@cuda_toolkit//:cub",
        "@cuda_toolkit//:cublas",
        "@cuda_toolkit//:cusolver",
    ],
    size = "small",
)
