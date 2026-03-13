load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cuda_headers",
    hdrs = glob([
        "include/**/*.h",
        "include/**/*.hpp",
        "include/**/*.cuh",
        "include/**/*.inl",
    ]),
    includes = ["include"],
)

cc_library(
    name = "thrust",
    deps = [":cuda_headers"],
)

cc_library(
    name = "cub",
    deps = [":cuda_headers"],
)

cc_library(
    name = "cublas",
    srcs = glob(["lib64/libcublas.so*"]),
    deps = [":cuda_headers"],
)

cc_library(
    name = "cusolver",
    srcs = glob(["lib64/libcusolver.so*"]),
    deps = [":cuda_headers"],
)

cc_library(
    name = "cusparse",
    srcs = glob(["lib64/libcusparse.so*"]),
    deps = [":cuda_headers"],
)
