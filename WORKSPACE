workspace(name = "shadesmar_gu")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Rules CC
http_archive(
    name = "rules_cc",
    urls = ["https://github.com/bazelbuild/rules_cc/releases/download/0.0.9/rules_cc-0.0.9.tar.gz"],
    sha256 = "2037875b9a4456dce4a79d112a8ae885bbc4aad968e6587dca6e64f3a0900cdf",
    strip_prefix = "rules_cc-0.0.9",
)
load("@rules_cc//cc:repositories.bzl", "rules_cc_dependencies")
rules_cc_dependencies()

# Rules Foreign CC (for building CMake projects)
http_archive(
    name = "rules_foreign_cc",
    sha256 = "476303bd0f1b04cc311fc258f1708a5f6ef82d3091e53fd1977fa20383425a6a",
    strip_prefix = "rules_foreign_cc-0.10.1",
    url = "https://github.com/bazelbuild/rules_foreign_cc/releases/download/0.10.1/rules_foreign_cc-0.10.1.tar.gz",
)
load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")
rules_foreign_cc_dependencies()

# Rules CUDA
http_archive(
    name = "rules_cuda",
    strip_prefix = "rules_cuda-v0.2.4",
    urls = ["https://github.com/bazel-contrib/rules_cuda/releases/download/v0.2.4/rules_cuda-v0.2.4.tar.gz"],
)
load("@rules_cuda//cuda:repositories.bzl", "rules_cuda_dependencies", "register_detected_cuda_toolchains")
rules_cuda_dependencies()
register_detected_cuda_toolchains()

# Hedron's Compile Commands
http_archive(
    name = "hedron_compile_commands",
    url = "https://github.com/mikael-s-persson/bazel-compile-commands-extractor/archive/fix/syntax_only_hdr_processing.tar.gz",
    strip_prefix = "bazel-compile-commands-extractor-fix-syntax_only_hdr_processing",
)
load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")
hedron_compile_commands_setup()
load("@hedron_compile_commands//:workspace_setup_transitive.bzl", "hedron_compile_commands_setup_transitive")
hedron_compile_commands_setup_transitive()
load("@hedron_compile_commands//:workspace_setup_transitive_transitive.bzl", "hedron_compile_commands_setup_transitive_transitive")
hedron_compile_commands_setup_transitive_transitive()
load("@hedron_compile_commands//:workspace_setup_transitive_transitive_transitive.bzl", "hedron_compile_commands_setup_transitive_transitive_transitive")
hedron_compile_commands_setup_transitive_transitive_transitive()

# GFlags
http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
    strip_prefix = "gflags-2.2.2",
    urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
)

# GLog
http_archive(
    name = "com_github_google_glog",
    sha256 = "8a83bf982f37bb70825df71a9709fa90ea9f4447fb3c099e1d720a439d88bad6",
    strip_prefix = "glog-0.6.0",
    urls = ["https://github.com/google/glog/archive/v0.6.0.tar.gz"],
)

# GTest
http_archive(
    name = "com_google_googletest",
    urls = ["https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz"],
    strip_prefix = "googletest-1.14.0",
)

# Eigen
new_local_repository(
    name = "eigen",
    build_file = "//third_party:eigen.BUILD",
    path = "/usr",
)

# OpenCV (built from source with CUDA — installed to /usr/local)
new_local_repository(
    name = "opencv",
    build_file = "//third_party:opencv.BUILD",
    path = "/usr/local",
)

# JSON
http_archive(
    name = "nlohmann_json",
    urls = ["https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz"],
    strip_prefix = "json-3.11.3",
    build_file = "@//third_party:nlohmann_json.BUILD",
)

# Python rules setup
http_archive(
    name = "rules_python",
    sha256 = "7a005bc3f1938fb04643b6637445d4d62c2dee545b2b18a17c6c561c67d2b174",
    strip_prefix = "rules_python-0.36.0",
    url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.36.0.zip",
)

# Initialize Python rules and load necessary functions
load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_toolchains")
py_repositories()

# Register Python toolchain
python_register_toolchains(
    name = "python3_10",
    python_version = "3.10",
)

# Python system library
new_local_repository(
    name = "python_linux",
    path = "/usr",
    build_file = "//third_party:python.BUILD",
)

# Numpy headers
new_local_repository(
    name = "numpy",
    path = "/usr/local/lib/python3.10/dist-packages",
    build_file = "//third_party:numpy.BUILD",
)

# matplotlib-cpp
http_archive(
    name = "matplotlib_cpp",
    build_file = "//third_party:matplotlib_cpp.BUILD",
    sha256 = "8109b1137051ac3bc15d757b0f2cfa8e1812e76c78312b4d8a6609d856046317",
    strip_prefix = "matplotlib-cpp-ef0383f1315d32e0156335e10b82e90b334f6d9f",
    urls = ["https://github.com/lava/matplotlib-cpp/archive/ef0383f1315d32e0156335e10b82e90b334f6d9f.tar.gz"],
)

# CUDA Toolkit (system installed)
new_local_repository(
    name = "cuda_toolkit",
    build_file = "//third_party:cuda_toolkit.BUILD",
    path = "/usr/local/cuda",
)

# Rerun C++ SDK (v0.30.2)
# Pre-built in the Docker container at /opt/rerun_sdk.
# Built via: cmake + make from the official rerun_cpp_sdk.zip.
# Contains: librerun_sdk.a, librerun_c.a, libarrow.a, headers.
new_local_repository(
    name = "rerun_cpp_sdk",
    build_file = "//third_party:rerun.BUILD",
    path = "/opt/rerun_sdk",
)
