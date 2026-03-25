load("@rules_cc//cc:defs.bzl", "cc_library")

# CPU-only OpenCV modules (used by most targets)
cc_library(
    name = "opencv",
    hdrs = glob(["include/opencv4/**/*.h*"]),
    includes = ["include/opencv4"],
    linkopts = [
        "-L/usr/local/lib",
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_highgui",
        "-lopencv_imgcodecs",
        "-lopencv_calib3d",
        "-lopencv_features2d",
        "-lopencv_flann",
        "-lopencv_videoio",
    ],
    visibility = ["//visibility:public"],
)

# OpenCV CUDA modules (for GPU-accelerated feature extraction & matching)
cc_library(
    name = "opencv_cuda",
    hdrs = glob(["include/opencv4/**/*.h*"]),
    includes = ["include/opencv4"],
    linkopts = [
        "-L/usr/local/lib",
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_highgui",
        "-lopencv_imgcodecs",
        "-lopencv_calib3d",
        "-lopencv_features2d",
        "-lopencv_flann",
        "-lopencv_videoio",
        "-lopencv_cudafeatures2d",
        "-lopencv_cudaimgproc",
        "-lopencv_cudawarping",
        "-lopencv_cudaarithm",
    ],
    visibility = ["//visibility:public"],
)
