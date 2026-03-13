load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "memory_map",
    hdrs = ["memory_map.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "graph_entity",
    hdrs = ["graph_entity.h"],
    visibility = ["//visibility:public"],
    deps = [":memory_map"],
)

cc_library(
    name = "ba_types",
    hdrs = ["ba_types.h"],
    visibility = ["//visibility:public"],
    deps = [":graph_entity"],
)

cc_library(
    name = "pose_graph_types",
    hdrs = ["pose_graph_types.h"],
    visibility = ["//visibility:public"],
    deps = [":graph_entity"],
)

cc_library(
    name = "landmark_types",
    hdrs = ["landmark_types.h"],
    visibility = ["//visibility:public"],
    deps = [":graph_entity"],
)
