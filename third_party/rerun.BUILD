# ==============================================================================
# Rerun C++ SDK v0.30.2 — Bazel BUILD file
#
# Pre-built in the Docker image at /opt/rerun_sdk.
# Contains:
#   lib/librerun_sdk.a         — C++ wrapper library
#   lib/librerun_c.a           — Rust core (pre-compiled static lib)
#   lib/libarrow.a             — Apache Arrow (static, built by Rerun's CMake)
#   lib/libarrow_bundled_dependencies.a — Arrow's bundled deps (mimalloc, etc.)
#   include/                   — All C++ and C headers (rerun.hpp, arrow/*, etc.)
# ==============================================================================

cc_library(
    name = "rerun_sdk",
    hdrs = glob(["include/**/*.hpp", "include/**/*.h"]),
    includes = ["include"],
    srcs = [
        "lib/librerun_sdk.a",
        "lib/librerun_c__linux_x64.a",
        "lib/libarrow.a",
        "lib/libarrow_bundled_dependencies.a",
    ],
    # Linux runtime deps for the Rust core (rerun_c)
    linkopts = [
        "-lm",
        "-ldl",
        "-lpthread",
    ],
    visibility = ["//visibility:public"],
)
