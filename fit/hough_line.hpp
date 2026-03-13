#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "fit/model.hpp"

namespace fit {

inline std::pair<std::optional<LineModel>, std::vector<Eigen::Vector2d>>
hough_transform(const std::vector<Eigen::Vector2d>& points,
                int theta_bins = 180, double rho_resolution = 0.5) {
  if (points.size() < 2) return {std::nullopt, {}};

  // Accumulator: theta_idx -> (rho_idx -> count)
  std::map<int, std::map<int, int>> accumulator;

  int best_votes = 0;
  int best_theta_idx = -1;
  int best_rho_idx = -1;

  double d_theta = M_PI / theta_bins;

  for (const auto& p : points) {
    for (int t = 0; t < theta_bins; ++t) {
      double theta = t * d_theta;
      double rho = p.x() * std::cos(theta) + p.y() * std::sin(theta);
      int rho_idx = std::round(rho / rho_resolution);

      accumulator[t][rho_idx]++;

      if (accumulator[t][rho_idx] > best_votes) {
        best_votes = accumulator[t][rho_idx];
        best_theta_idx = t;
        best_rho_idx = rho_idx;
      }
    }
  }

  if (best_votes < 2) return {std::nullopt, {}};

  double best_theta = best_theta_idx * d_theta;
  double best_rho = best_rho_idx * rho_resolution;

  // Convert to ax + by + c = 0
  // x cos(theta) + y sin(theta) - rho = 0
  double a = std::cos(best_theta);
  double b = std::sin(best_theta);
  double c_val = -best_rho;

  // Collect inliers
  std::vector<Eigen::Vector2d> inliers;
  inliers.reserve(best_votes);  // Approximation

  // We need a threshold to determine inliers, as Hough is discrete.
  // Using rho_resolution as a heuristic threshold.
  double threshold = rho_resolution;

  for (const auto& p : points) {
    double dist = std::abs(a * p.x() + b * p.y() + c_val);
    if (dist < threshold) {
      inliers.push_back(p);
    }
  }

  return {LineModel{a, b, c_val}, inliers};
}

}  // namespace fit
