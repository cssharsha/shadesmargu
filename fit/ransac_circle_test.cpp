#include "fit/ransac_circle.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

// Define M_PI if not available (standard in cmath but sometimes guarded)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool test_perfect_circle() {
  std::cout << "Running perfect circle test..." << std::endl;
  std::vector<Eigen::Vector2d> points;
  double radius = 5.0;
  Eigen::Vector2d center(0.0, 0.0);

  // Generate points on a circle
  // We ensure we have diametrically opposite points for the current RANSAC
  // implementation which assumes two points form a diameter.
  int num_points = 20;
  for (int i = 0; i < num_points; ++i) {
    double theta = 2.0 * M_PI * i / num_points;
    points.emplace_back(center.x() + radius * std::cos(theta),
                        center.y() + radius * std::sin(theta));
  }

  // Add a few outliers
  points.emplace_back(10.0, 10.0);
  points.emplace_back(-10.0, -10.0);

  auto result = fit::circle::ransac(points, 100);

  if (!result.first) {
    std::cerr << "Circle RANSAC failed to find a model." << std::endl;
    return false;
  }

  const auto& model = *result.first;
  std::cout << "Found Circle: Center(" << model.a << ", " << model.b
            << "), Radius " << model.r << std::endl;

  // Verification
  if (std::abs(model.a - center.x()) > 1e-1 ||
      std::abs(model.b - center.y()) > 1e-1) {
    std::cerr << "Center mismatch. Expected (0,0)" << std::endl;
    return false;
  }

  if (std::abs(model.r - radius) > 1e-1) {
    std::cerr << "Radius mismatch. Expected 5.0, got " << model.r << std::endl;
    return false;
  }

  // Check inliers count (should be 20)
  // The implementation returns indices.
  if (result.second.size() < 20) {
    std::cerr << "Too few inliers. Expected at least 20, got "
              << result.second.size() << std::endl;
    return false;
  }

  return true;
}

int main() {
  if (!test_perfect_circle()) {
    return 1;
  }

  std::cout << "All circle tests passed!" << std::endl;
  return 0;
}
