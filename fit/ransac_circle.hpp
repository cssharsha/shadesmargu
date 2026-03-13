#pragma once

#include <Eigen/Dense>
#include <optional>
#include <random>
#include <vector>

#include "model.hpp"

namespace fit {
namespace circle {

inline std::pair<std::optional<Circle>, std::vector<int>> ransac(
    const std::vector<Eigen::Vector2d>& points, int max_iterations,
    double threshold = 1e-2) {
  if (points.size() < 2) {
    return {std::nullopt, {}};
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, points.size() - 1);

  Circle best = {0, 0, 0};
  int max_inliers = -1;
  std::vector<int> best_inliers;

  for (int i = 0; i < max_iterations; ++i) {
    int idx1 = dis(gen);
    int idx2 = dis(gen);
    int retry = 0;
    while (idx1 == idx2 && retry < 10) {
      idx2 = dis(gen);
      retry++;
    }
    if (idx1 == idx2) continue;

    const Eigen::Vector2d& p1 = points[idx1];
    const Eigen::Vector2d& p2 = points[idx2];

    auto mid = p1 + ((p2 - p1) / 2.);
    auto a = mid.x();
    auto b = mid.y();

    auto r_sq = (p1.x() - a) * (p1.x() - a) + (p1.y() - b) * (p1.y() - b);
    auto r = std::sqrt(r_sq);

    int inlier_count = 0;
    std::vector<int> current_inliers;
    for (int idx = 0; idx < points.size(); idx++) {
      auto dist_sq = (points[idx].x() - a) * (points[idx].x() - a) +
                     (points[idx].y() - b) * (points[idx].y() - b);
      auto dist = std::abs(std::sqrt(dist_sq) - r);
      if (dist < threshold) {
        inlier_count++;
        current_inliers.push_back(idx);
      }
    }

    if (inlier_count > max_inliers) {
      max_inliers = inlier_count;
      best.a = a;
      best.b = b;
      best.r = r;
      best_inliers = std::move(current_inliers);
    }
  }

  if (max_inliers == -1) return {std::nullopt, {}};

  return {best, best_inliers};
}

}  // namespace circle
}  // namespace fit
