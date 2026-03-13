load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
)

cmake(
    name = "glog",
    lib_source = ":all_srcs",
    out_shared_libs = ["libglog.so"],
    cache_entries = {
        "WITH_GFLAGS": "ON",
        "BUILD_SHARED_LIBS": "ON",
        "BUILD_TESTING": "OFF",
    },
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
    visibility = ["//visibility:public"],
)
