#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <utility>
#include <vector>

namespace filters {

// The motion model is just a unicycle one where the control inputs are
// linear and angular velocity. In one of the interviews I was asked to do
// a control input of distance and the state having just x and y. I dont
// remember it exactly but Im trying to hunt it down ever since.
struct UnicycleMotionModel {
  // Constructor to initialize the process noise covariance matrix R
  // This allows the noise parameters to be set during initialization
  explicit UnicycleMotionModel(const Eigen::MatrixXd& process_noise)
      : R(process_noise) {}

  std::pair<Eigen::VectorXd, Eigen::MatrixXd> predict(
      const Eigen::VectorXd& state, const Eigen::MatrixXd& P,
      const Eigen::VectorXd& input, double dt) const;

 private:
  Eigen::MatrixXd R;
};

struct RangeObservationModel {
  explicit RangeObservationModel(const Eigen::MatrixXd& measurement_noise);
  Eigen::VectorXd predictObservation(const Eigen::VectorXd& state) const;

  Eigen::MatrixXd getJacobian(const Eigen::VectorXd& state) const;
  Eigen::MatrixXd getMeasurementNoise() const { return Q; }

  std::pair<Eigen::VectorXd, Eigen::MatrixXd> update(
      const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance,
      const Eigen::VectorXd& measurement) const;

 private:
  Eigen::MatrixXd Q;
  std::vector<Eigen::Vector2d> landmarks;
};
}  // namespace filters
