load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "opencv",
    hdrs = glob(["include/opencv4/**/*.h*"]),
    includes = ["include/opencv4"],
    linkopts = [
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
