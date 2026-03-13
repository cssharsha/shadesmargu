#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "filters/kalman/ekf.hpp"
#include "filters/kalman/models.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
namespace fs = std::filesystem;

using namespace filters;
using Eigen::MatrixXd;
using Eigen::VectorXd;

std::pair<std::vector<double>, std::vector<double>> get_ellipse_points(
    const VectorXd& state, const MatrixXd& covariance,
    double chi2_val = 5.991) {
  // chi2_val = 5.991 for 95% confidence interval with 2 degrees of freedom
  MatrixXd cov2d = covariance.block(0, 0, 2, 2);
  Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver(cov2d);
  if (eigensolver.info() != Eigen::Success) {
    return {};
  }

  VectorXd eigenvalues = eigensolver.eigenvalues();
  MatrixXd eigenvectors = eigensolver.eigenvectors();

  double angle = std::atan2(eigenvectors(1, 0), eigenvectors(0, 0));
  double a = std::sqrt(chi2_val * eigenvalues(0));
  double b = std::sqrt(chi2_val * eigenvalues(1));

  std::vector<double> x_points, y_points;
  for (double t = 0; t <= 2 * M_PI; t += 0.1) {
    double x_local = a * std::cos(t);
    double y_local = b * std::sin(t);

    // Rotate and translate
    double x_global =
        state(0) + std::cos(angle) * x_local - std::sin(angle) * y_local;
    double y_global =
        state(1) + std::sin(angle) * x_local + std::cos(angle) * y_local;

    x_points.push_back(x_global);
    y_points.push_back(y_global);
  }
  return {x_points, y_points};
}

std::vector<VectorXd> read_file(const std::string& filename, int cols) {
  std::vector<VectorXd> data;
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Failed to open " << filename << std::endl;
    exit(1);
  }
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    VectorXd vec(cols);
    for (int i = 0; i < cols; ++i) {
      ss >> vec(i);
    }
    data.push_back(vec);
  }
  return data;
}

int main(int argc, char* argv[]) {
  std::string output_dir = "/data/se/filters";
  if (argc > 1) {
    output_dir = argv[1];
  }

  if (!fs::exists(output_dir)) {
    try {
      fs::create_directories(output_dir);
    } catch (const std::exception& e) {
      std::cerr << "Error creating directory " << output_dir << ": " << e.what()
                << std::endl;
      // Fallback to current directory or /tmp if permission denied, or just
      // exit.
      std::cerr << "Falling back to saving in current directory." << std::endl;
      output_dir = ".";
    }
  }

  auto inputs = read_file("filters/ekf/inputs.txt", 2);
  auto measurements = read_file("filters/ekf/sensor_readings.txt", 3);

  if (inputs.size() < measurements.size()) {
    std::cerr << "Not enough inputs for measurements." << std::endl;
    return 1;
  }

  VectorXd mu(3);
  mu << 0, 0, 0;

  MatrixXd sigma = MatrixXd::Identity(3, 3) * 0.001;

  EKF ekf(mu, sigma);

  MatrixXd R = MatrixXd::Identity(3, 3);
  R.diagonal() << 0.01, 0.01, 0.001;
  UnicycleMotionModel motion_model(R);

  MatrixXd Q = MatrixXd::Identity(3, 3) * 0.001;
  RangeObservationModel obs_model(Q);

  double dt = 0.5;

  std::vector<double> x_est, y_est;
  std::vector<double> x_true,
      y_true;  // We don't have ground truth in inputs/readings, so we'll just
               // plot estimate

  // Landmarks
  std::vector<double> lx = {5, 4, -3};
  std::vector<double> ly = {5, 7, 2};

  std::vector<std::pair<std::vector<double>, std::vector<double>>> ellipses;

  for (size_t i = 0; i < measurements.size(); ++i) {
    ekf.predict(motion_model, inputs[i], dt);
    ekf.update(obs_model, measurements[i]);

    VectorXd state = ekf.state();
    x_est.push_back(state(0));
    y_est.push_back(state(1));

    if (i % 5 == 0) {
      ellipses.push_back(get_ellipse_points(state, ekf.covariance()));
    }

    std::cout << "Calculated mu:" << std::endl;
    std::cout << ekf.state() << std::endl;
    std::cout << "Calculated sigma:" << std::endl;
    std::cout << ekf.covariance() << std::endl;
  }

  plt::figure();
  plt::plot(x_est, y_est, "b-");
  plt::plot(lx, ly, "ro");  // Plot landmarks

  for (const auto& ellipse : ellipses) {
    if (!ellipse.first.empty()) {
      plt::plot(ellipse.first, ellipse.second, "g-");
    }
  }

  plt::title("EKF Localization with Covariance");
  plt::xlabel("X");
  plt::ylabel("Y");

  std::string output_file = output_dir + "/ekf_localization.png";
  std::cout << "Saving visualization to " << output_file << std::endl;
  plt::save(output_file);

  return 0;
}
