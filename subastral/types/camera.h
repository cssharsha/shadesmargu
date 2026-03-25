#pragma once

#include <opencv2/core.hpp>

namespace substral {

/// Pinhole camera intrinsics with radial-tangential distortion (OpenCV model).
struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  // Distortion coefficients (OpenCV ordering: k1, k2, p1, p2, k3)
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  double k3 = 0.0;

  /// Returns the 3x3 camera matrix K.
  cv::Mat cameraMatrix() const {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    return K;
  }

  /// Returns the 5x1 distortion coefficient vector.
  cv::Mat distCoeffs() const {
    cv::Mat d = cv::Mat::zeros(5, 1, CV_64F);
    d.at<double>(0) = k1;
    d.at<double>(1) = k2;
    d.at<double>(2) = p1;
    d.at<double>(3) = p2;
    d.at<double>(4) = k3;
    return d;
  }

  /// True if all distortion coefficients are zero.
  bool isUndistorted() const {
    return k1 == 0.0 && k2 == 0.0 && p1 == 0.0 && p2 == 0.0 && k3 == 0.0;
  }
};

}  // namespace substral
