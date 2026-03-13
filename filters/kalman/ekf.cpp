#include "filters/kalman/ekf.hpp"

namespace filters {

EKF::EKF() {}

EKF::EKF(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance)
    : state_(state), covariance_(covariance) {}

}  // namespace filters
