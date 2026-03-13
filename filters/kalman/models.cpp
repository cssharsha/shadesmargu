#include "filters/kalman/models.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace filters {

// Input is a vector of [v, w]. The state currently is [x, y, theta].
// Again I was asked to use a matrix form to do the predict step but
// I doubt there is an elegant one.
std::pair<Eigen::VectorXd, Eigen::MatrixXd> UnicycleMotionModel::predict(
    const Eigen::VectorXd& state, const Eigen::MatrixXd& P,
    const Eigen::VectorXd& input, double dt) const {
  double theta = state(2);
  double v = input(0);

  // Just making it matrix form just for the fun of it.
  // B matrix maps control input to state change
  // [ cos(theta), 0 ]
  // [ sin(theta), 0 ]
  // [ 0,          1 ]
  Eigen::MatrixXd B(3, 2);
  B << std::cos(theta), 0, std::sin(theta), 0, 0, 1;

  Eigen::VectorXd next_state = state + (B * input) * dt;

  // Normalize theta to [-pi, pi]
  while (next_state(2) > M_PI) next_state(2) -= 2 * M_PI;
  while (next_state(2) < -M_PI) next_state(2) += 2 * M_PI;

  // Jacobian Fx = df/dx
  // [ 1, 0, -v*dt*sin(theta) ]
  // [ 0, 1,  v*dt*cos(theta) ]
  // [ 0, 0,  1               ]
  Eigen::MatrixXd Fx = Eigen::MatrixXd::Identity(3, 3);
  Fx(0, 2) = -v * dt * std::sin(theta);
  Fx(1, 2) = v * dt * std::cos(theta);

  // P_{t+1} = Fx * P_t * Fx^T + R
  Eigen::MatrixXd next_covariance = Fx * P * Fx.transpose() + R;

  return {next_state, next_covariance};
}

RangeObservationModel::RangeObservationModel(
    const Eigen::MatrixXd& measurement_noise)
    : Q(measurement_noise) {
  landmarks.push_back(Eigen::Vector2d(5, 5));
  landmarks.push_back(Eigen::Vector2d(4, 7));
  landmarks.push_back(Eigen::Vector2d(-3, 2));
}

// Calculate expected measurement h(x)
Eigen::VectorXd RangeObservationModel::predictObservation(
    const Eigen::VectorXd& state) const {
  Eigen::VectorXd h(landmarks.size());
  for (size_t i = 0; i < landmarks.size(); ++i) {
    double dx = state(0) - landmarks[i](0);
    double dy = state(1) - landmarks[i](1);
    h(i) = std::sqrt(dx * dx + dy * dy);
  }
  return h;
}

// The Jacobian Matrix H is defined as the partial derivative of the observation
// function h(x) with respect to the state vector x = [x, y, theta]^T.
// H_ij = dh_i / dx_j
//
// For a range measurement to a landmark L_i = (mx, my):
// h_i(x) = sqrt((x - mx)^2 + (y - my)^2)
//
// The partial derivatives are derived as follows:
// dh_i/dx = (1/2) * ((x - mx)^2 + (y - my)^2)^(-1/2) * 2(x - mx)
//         = (x - mx) / h_i(x)
//
// dh_i/dy = (1/2) * ((x - mx)^2 + (y - my)^2)^(-1/2) * 2(y - my)
//         = (y - my) / h_i(x)
// dh_i/dtheta = 0
Eigen::MatrixXd RangeObservationModel::getJacobian(
    const Eigen::VectorXd& state) const {
  Eigen::MatrixXd H(landmarks.size(), state.size());
  for (size_t i = 0; i < landmarks.size(); ++i) {
    double dx = state(0) - landmarks[i](0);
    double dy = state(1) - landmarks[i](1);
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 1e-6) dist = 1e-6;  // Avoid division by zero

    H(i, 0) = dx / dist;
    H(i, 1) = dy / dist;
    // Derivative w.r.t theta is 0
    for (int j = 2; j < state.size(); ++j) {
      H(i, j) = 0.0;
    }
  }
  return H;
}

// Update step: Computes the Kalman gain and updates state and covariance
std::pair<Eigen::VectorXd, Eigen::MatrixXd> RangeObservationModel::update(
    const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance,
    const Eigen::VectorXd& measurement) const {
  Eigen::VectorXd h = predictObservation(state);
  Eigen::MatrixXd H = getJacobian(state);
  Eigen::MatrixXd S = H * covariance * H.transpose() + Q;
  // Compute the Kalman Gain. H is the Observation Matrix (Jacobian).
  Eigen::MatrixXd K = covariance * H.transpose() * S.inverse();
  Eigen::VectorXd y = measurement - h;
  Eigen::VectorXd next_state = state + K * y;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state.size(), state.size());
  Eigen::MatrixXd next_covariance = (I - K * H) * covariance;
  return {next_state, next_covariance};
}
}  // namespace filters
