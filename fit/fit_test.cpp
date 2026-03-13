#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "fit/hough_line.hpp"
#include "fit/ransac_line.hpp"

bool verify_model(const fit::LineModel& model,
                  const std::vector<Eigen::Vector2d>& inliers,
                  const std::string& method_name) {
  std::cout << "Verifying " << method_name << " model: " << model.a << "x + "
            << model.b << "y + " << model.c << " = 0" << std::endl;

  int inliers_count = 0;
  // Check inliers (0,0) to (49,49)
  for (int i = 0; i < 50; ++i) {
    double dist = std::abs(model.a * i + model.b * i + model.c);
    if (dist < 1.0) {
      inliers_count++;
    }
  }

  std::cout << "Model Inliers count (expected 50): " << inliers_count
            << std::endl;
  if (inliers_count < 45) {  // Allow some slack
    std::cerr << method_name << " failed to capture enough inliers."
              << std::endl;
    return false;
  }

  std::cout << "Returned Inliers count: " << inliers.size() << std::endl;
  // Verify returned inliers match expected count roughly
  if (inliers.size() < 45) {
    std::cerr << method_name << " returned too few inliers." << std::endl;
    return false;
  }

  int outliers_included = 0;
  // Check outliers (0,10) to (19,29)
  for (int i = 0; i < 20; ++i) {
    double dist = std::abs(model.a * i + model.b * (i + 10) + model.c);
    if (dist < 1.0) {
      outliers_included++;
    }
  }

  std::cout << "Outliers included (expected 0): " << outliers_included
            << std::endl;
  if (outliers_included > 2) {
    std::cerr << method_name << " captured too many outliers." << std::endl;
    return false;
  }

  return true;
}

bool test_specific_points() {
  std::cout << "\nRunning specific points test..." << std::endl;
  std::vector<Eigen::Vector2d> points = {{1, 1}, {3, 2}, {5, 3},
                                         {4, 1}, {2, 3}, {1, 4}};

  // Expected line: x - 2y + 1 = 0
  // Normalized: 1/sqrt(5) x - 2/sqrt(5) y + 1/sqrt(5) = 0
  // a approx 0.447, b approx -0.894, c approx 0.447

  // RANSAC
  // Use strict threshold
  auto result = fit::ransac(points, 50, 0.1);

  if (!result.first) {
    std::cerr << "Specific points RANSAC failed." << std::endl;
    return false;
  }

  const auto& model = *result.first;
  std::cout << "Specific RANSAC Model: " << model.a << "x + " << model.b
            << "y + " << model.c << " = 0" << std::endl;
  std::cout << "Inliers found: " << result.second.size() << std::endl;

  if (result.second.size() != 3) {
    std::cerr << "Expected 3 inliers, got " << result.second.size()
              << std::endl;
    return false;
  }

  // Check coefficients (up to sign flip)
  // We check if (a,b) is parallel to (1, -2)
  // Cross product of normal vectors should be near zero.
  // (a,b) x (1,-2) = -2a - b
  double cross_prod = -2 * model.a - model.b;
  if (std::abs(cross_prod) > 1e-2) {
    std::cerr << "Model orientation incorrect. Expected roughly parallel to "
                 "(1, -2). Cross prod: "
              << cross_prod << std::endl;
    return false;
  }

  return true;
}

int main() {
  if (!test_specific_points()) return 1;

  std::vector<Eigen::Vector2d> points;
  for (int i = 0; i < 50; ++i) points.emplace_back(i, i);
  for (int i = 0; i < 20; ++i) points.emplace_back(i, i + 10);

  // RANSAC
  auto ransac_result_pair = fit::ransac(points, 100, 0.1);
  if (!ransac_result_pair.first) {
    std::cerr << "RANSAC failed." << std::endl;
    return 1;
  }
  if (!verify_model(*ransac_result_pair.first, ransac_result_pair.second,
                    "RANSAC"))
    return 1;

  // Hough
  // 360 bins for theta -> 0.5 degrees
  // 0.5 resolution for rho
  auto hough_result_pair = fit::hough_transform(points, 360, 0.5);
  if (!hough_result_pair.first) {
    std::cerr << "Hough failed." << std::endl;
    return 1;
  }
  if (!verify_model(*hough_result_pair.first, hough_result_pair.second,
                    "Hough"))
    return 1;

  std::cout << "All tests passed!" << std::endl;
  return 0;
}
