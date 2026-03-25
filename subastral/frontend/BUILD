load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")

# Abstract interfaces (header-only)
cc_library(
    name = "feature_extractor",
    hdrs = ["feature_extractor.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "@opencv//:opencv",
    ],
)

cc_library(
    name = "feature_matcher",
    hdrs = ["feature_matcher.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "@opencv//:opencv",
    ],
)

# CPU implementations
cc_library(
    name = "feature_extractor_cpu",
    srcs = ["feature_extractor_cpu.cpp"],
    hdrs = ["feature_extractor_cpu.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":feature_extractor",
        "@opencv//:opencv",
    ],
)

cc_library(
    name = "feature_matcher_cpu",
    srcs = ["feature_matcher_cpu.cpp"],
    hdrs = ["feature_matcher_cpu.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":feature_matcher",
        "@opencv//:opencv",
    ],
)

# GPU implementations
cuda_library(
    name = "feature_extractor_gpu",
    srcs = ["feature_extractor_gpu.cu"],
    hdrs = ["feature_extractor_gpu.cuh"],
    visibility = ["//visibility:public"],
    deps = [
        ":feature_extractor",
        "@opencv//:opencv_cuda",
    ],
)

cuda_library(
    name = "feature_matcher_gpu",
    srcs = ["feature_matcher_gpu.cu"],
    hdrs = ["feature_matcher_gpu.cuh"],
    visibility = ["//visibility:public"],
    deps = [
        ":feature_matcher",
        "@opencv//:opencv_cuda",
    ],
)

# GPU image preprocessor (upload + cvtColor + remap on device)
cuda_library(
    name = "image_preprocessor_gpu",
    srcs = ["image_preprocessor_gpu.cu"],
    hdrs = ["image_preprocessor_gpu.cuh"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/types:types",
        "@opencv//:opencv_cuda",
    ],
)

# Two-view geometry (CPU only — RANSAC is inherently sequential)
cc_library(
    name = "two_view_geometry",
    srcs = ["two_view_geometry.cpp"],
    hdrs = ["two_view_geometry.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/types:types",
        "@eigen//:eigen",
        "@opencv//:opencv",
    ],
)

# IMU pre-integration (Forster et al.)
cc_library(
    name = "imu_preintegrator",
    srcs = ["imu_preintegrator.cpp"],
    hdrs = ["imu_preintegrator.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/backend/lie:so3",
        "//subastral/loader:dataset_types",
        "@eigen//:eigen",
    ],
)

# Gravity direction estimation from static accelerometer data
cc_library(
    name = "gravity_initializer",
    srcs = ["gravity_initializer.cpp"],
    hdrs = ["gravity_initializer.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/loader:dataset_types",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "imu_preintegrator_test",
    srcs = ["imu_preintegrator_test.cpp"],
    deps = [
        ":imu_preintegrator",
        "//subastral/backend/lie:so3",
        "@com_google_googletest//:gtest_main",
        "@eigen//:eigen",
    ],
    size = "small",
)

# Depth-based map initialization (RGB-D)
cc_library(
    name = "depth_initializer",
    srcs = ["depth_initializer.cpp"],
    hdrs = ["depth_initializer.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//subastral/types:types",
        "@eigen//:eigen",
        "@opencv//:opencv",
    ],
)

# IMU-aided initialization (gravity alignment + pre-integration for frame selection)
cc_library(
    name = "imu_initializer",
    srcs = ["imu_initializer.cpp"],
    hdrs = ["imu_initializer.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":gravity_initializer",
        ":imu_preintegrator",
        "//subastral/backend/lie:so3",
        "//subastral/loader:dataset_types",
        "//subastral/types:types",
        "@eigen//:eigen",
    ],
)

# Test: CPU + GPU feature extraction and matching on a real TUM image
cc_test(
    name = "feature_extractor_test",
    srcs = ["feature_extractor_test.cpp"],
    deps = [
        ":feature_extractor_cpu",
        ":feature_matcher_cpu",
        "@opencv//:opencv",
        "@com_google_googletest//:gtest_main",
    ],
    size = "small",
)

cc_test(
    name = "two_view_geometry_test",
    srcs = ["two_view_geometry_test.cpp"],
    deps = [
        ":feature_extractor_cpu",
        ":feature_matcher_cpu",
        ":two_view_geometry",
        "//subastral/loader:tum_loader",
        "@eigen//:eigen",
        "@opencv//:opencv",
        "@com_google_googletest//:gtest_main",
    ],
    size = "small",
)

# Standalone diagnostic: analyze two-view triangulation quality vs GT
cc_binary(
    name = "two_view_diagnostic",
    srcs = ["two_view_diagnostic.cpp"],
    deps = [
        ":feature_extractor_cpu",
        ":feature_matcher_cpu",
        ":two_view_geometry",
        "//subastral/loader:tum_loader",
        "@eigen//:eigen",
        "@opencv//:opencv",
    ],
)

cc_test(
    name = "feature_gpu_test",
    srcs = ["feature_gpu_test.cpp"],
    deps = [
        ":feature_extractor_cpu",
        ":feature_extractor_gpu",
        ":feature_matcher_cpu",
        ":feature_matcher_gpu",
        "@opencv//:opencv_cuda",
        "@com_google_googletest//:gtest_main",
    ],
    size = "small",
)

cc_test(
    name = "image_preprocessor_gpu_test",
    srcs = ["image_preprocessor_gpu_test.cpp"],
    deps = [
        ":feature_extractor_cpu",
        ":feature_extractor_gpu",
        ":image_preprocessor_gpu",
        "//subastral/loader:tum_loader",
        "@opencv//:opencv_cuda",
        "@com_google_googletest//:gtest_main",
    ],
    size = "small",
)
