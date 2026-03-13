#pragma once

#include <Eigen/Dense>
#include <utility>

namespace filters {

class EKF {
 public:
  EKF();
  EKF(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance);

  template <typename MotionModel>
  void predict(const MotionModel& model, const Eigen::VectorXd& input,
               double dt) {
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> predicted =
        model.predict(state_, covariance_, input, dt);
    state_ = predicted.first;
    covariance_ = predicted.second;
  }
  template <typename ObservationModel>
  void update(const ObservationModel& model,
              const Eigen::VectorXd& measurement) {
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> updated =
        model.update(state_, covariance_, measurement);
    state_ = updated.first;
    covariance_ = updated.second;
  }

  const Eigen::VectorXd& state() const { return state_; }
  const Eigen::MatrixXd& covariance() const { return covariance_; }

 private:
  Eigen::VectorXd state_;
  Eigen::MatrixXd covariance_;

  bool bar_{false};
};
}  // namespace filters
