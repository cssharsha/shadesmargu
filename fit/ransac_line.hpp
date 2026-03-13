#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <optional>
#include <random>
#include <utility>
#include <vector>

#include "fit/model.hpp"

namespace fit {

inline std::pair<std::optional<LineModel>, std::vector<Eigen::Vector2d>> ransac(
    const std::vector<Eigen::Vector2d>& points, int max_iterations,
    double threshold) {
  if (points.size() < 2) {
    return {std::nullopt, {}};
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, points.size() - 1);

  LineModel best_model = {0, 0, 0};
  int max_inliers = -1;
  std::vector<Eigen::Vector2d> best_inliers;

  for (int i = 0; i < max_iterations; ++i) {
    int idx1 = dis(gen);
    int idx2 = dis(gen);

    // Try to pick distinct points
    int retry = 0;
    while (idx1 == idx2 && retry < 10) {
      idx2 = dis(gen);
      retry++;
    }
    if (idx1 == idx2) continue;

    const Eigen::Vector2d& p1 = points[idx1];
    const Eigen::Vector2d& p2 = points[idx2];

    if ((p1 - p2).norm() < 1e-6) continue;

    // Line equation ax + by + c = 0
    // Vector along line is (dx, dy)
    double dx = p2.x() - p1.x();
    double dy = p2.y() - p1.y();

    // Normal is (-dy, dx)
    double a = -dy;
    double b = dx;

    // Normalize
    double norm = std::sqrt(a * a + b * b);
    if (norm < 1e-9) continue;
    a /= norm;
    b /= norm;

    double c_val = -(a * p1.x() + b * p1.y());

    int inliers_count = 0;
    std::vector<Eigen::Vector2d> current_inliers;
    current_inliers.reserve(points.size());

    for (const auto& p : points) {
      double dist = std::abs(a * p.x() + b * p.y() + c_val);
      if (dist < threshold) {
        inliers_count++;
        current_inliers.push_back(p);
      }
    }

    if (inliers_count > max_inliers) {
      max_inliers = inliers_count;
      best_model = {a, b, c_val};
      best_inliers = std::move(current_inliers);
    }
  }

  if (max_inliers == -1) return {std::nullopt, {}};
  return {best_model, best_inliers};
}

}  // namespace fit
