#include "subastral/backend/solver/lm_solver.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

#include "subastral/backend/ops/project_with_jacobian_cpu.hpp"
#include "subastral/backend/ops/projection_cpu.hpp"
#include "subastral/backend/solver/schur.hpp"

namespace substral {
namespace backend {
namespace solver {

// =============================================================================
// Helper: evaluate loss function rho(s) for a given squared residual norm s
// =============================================================================
static double evalRho(LossType type, double s, double param) {
  switch (type) {
    case LossType::HUBER: {
      HuberLoss loss(param);
      return loss.rho(s);
    }
    case LossType::CAUCHY: {
      CauchyLoss loss(param);
      return loss.rho(s);
    }
    case LossType::TRIVIAL:
    default:
      return TrivialLoss::rho(s);
  }
}

static double evalWeight(LossType type, double s, double param) {
  switch (type) {
    case LossType::HUBER: {
      HuberLoss loss(param);
      return loss.weight(s);
    }
    case LossType::CAUCHY: {
      CauchyLoss loss(param);
      return loss.weight(s);
    }
    case LossType::TRIVIAL:
    default:
      return TrivialLoss::weight(s);
  }
}

// =============================================================================
// Helper: compute robust cost = 0.5 * sum rho(||r_i||^2)
// =============================================================================
//
// For TrivialLoss: rho(s) = s, so cost = 0.5 * sum s_i (standard L2)
// For Huber/Cauchy: rho(s) < s for large s (outlier down-weighting)
//
static double computeCost(BAProblem& problem, LossType loss_type,
                          double loss_param) {
  double total = 0.0;
  for (auto& obs : problem.observations) {
    const double* cam = problem.cameras[obs->get_camera_id()]->data();
    const double* pt = problem.points[obs->get_point_id()]->data();
    double predicted[2];
    ops::project_cpu(cam, pt, predicted);
    double rx = predicted[0] - obs->data()[0];
    double ry = predicted[1] - obs->data()[1];
    double s = rx * rx + ry * ry;
    total += evalRho(loss_type, s, loss_param);
  }
  return 0.5 * total;
}

// =============================================================================
// Helper: compute residuals and Jacobians for all observations
// =============================================================================
static void computeResidualsAndJacobians(BAProblem& problem,
                                         std::vector<double>& residuals,
                                         std::vector<double>& J_cameras,
                                         std::vector<double>& J_points) {
  int num_obs = static_cast<int>(problem.observations.size());
  residuals.resize(num_obs * RES_DIM);
  J_cameras.resize(num_obs * RES_DIM * CAM_DIM);
  J_points.resize(num_obs * RES_DIM * PT_DIM);

  for (int k = 0; k < num_obs; ++k) {
    const double* cam =
        problem.cameras[problem.observations[k]->get_camera_id()]->data();
    const double* pt =
        problem.points[problem.observations[k]->get_point_id()]->data();

    double predicted[2];
    ops::projectWithJacobianCPU(cam, pt, predicted,
                                &J_cameras[k * RES_DIM * CAM_DIM],
                                &J_points[k * RES_DIM * PT_DIM]);

    residuals[k * RES_DIM + 0] =
        predicted[0] - problem.observations[k]->data()[0];
    residuals[k * RES_DIM + 1] =
        predicted[1] - problem.observations[k]->data()[1];
  }
}

// =============================================================================
// Helper: apply parameter update (or rollback)
// =============================================================================
static void applyUpdate(BAProblem& problem,
                        const Eigen::VectorXd& delta_cameras,
                        const Eigen::VectorXd& delta_points) {
  // Update cameras
  for (int i = 0; i < static_cast<int>(problem.cameras.size()); ++i) {
    double* cam = problem.cameras[i]->mutable_data();
    for (int j = 0; j < CAM_DIM; ++j) {
      cam[j] += delta_cameras[i * CAM_DIM + j];
    }
  }
  // Update points
  for (int j = 0; j < static_cast<int>(problem.points.size()); ++j) {
    double* pt = problem.points[j]->mutable_data();
    for (int d = 0; d < PT_DIM; ++d) {
      pt[d] += delta_points[j * PT_DIM + d];
    }
  }
}

// =============================================================================
// Levenberg-Marquardt Solver
// =============================================================================

LMResult solveLM(BAProblem& problem, const LMConfig& config) {
  LMResult result;
  result.converged = false;
  result.iterations = 0;

  int num_cameras = static_cast<int>(problem.cameras.size());
  int num_points = static_cast<int>(problem.points.size());
  int num_obs = static_cast<int>(problem.observations.size());

  if (num_obs == 0) {
    result.initial_cost = 0;
    result.final_cost = 0;
    result.converged = true;
    result.termination_reason = "No observations";
    return result;
  }

  double lambda = config.initial_lambda;
  double current_cost =
      computeCost(problem, config.loss_type, config.loss_param);
  result.initial_cost = current_cost;

  if (config.verbose) {
    std::cout << "LM: initial cost = " << current_cost
              << " (RMS = " << std::sqrt(2.0 * current_cost / num_obs) << ")"
              << std::endl;
  }

  // Save original parameters for rollback
  std::vector<double> saved_cameras(num_cameras * CAM_DIM);
  std::vector<double> saved_points(num_points * PT_DIM);

  for (int iter = 0; iter < config.max_iterations; ++iter) {
    result.iterations = iter + 1;

    // ---- Early termination: cost is already zero ----
    // This can happen when the robust loss drives cost to exactly 0.
    // No further improvement is possible.
    if (current_cost <= 0.0) {
      result.converged = true;
      result.termination_reason = "Cost is zero";
      break;
    }

    // ---- Step 1: Compute residuals and Jacobians ----
    std::vector<double> residuals, J_cameras, J_points;
    computeResidualsAndJacobians(problem, residuals, J_cameras, J_points);

    // ---- Step 1b: Apply IRLS weighting for robust loss ----
    // Scale each observation's residual and Jacobian by sqrt(w_k)
    // where w_k = rho'(s_k) and s_k = ||r_k||^2.
    // For TrivialLoss, w_k = 1 so this is a no-op.
    if (config.loss_type != LossType::TRIVIAL) {
      for (int k = 0; k < num_obs; ++k) {
        double rx = residuals[k * RES_DIM + 0];
        double ry = residuals[k * RES_DIM + 1];
        double s = rx * rx + ry * ry;
        double w = evalWeight(config.loss_type, s, config.loss_param);
        double sqrt_w = std::sqrt(w);

        // Scale residual
        residuals[k * RES_DIM + 0] *= sqrt_w;
        residuals[k * RES_DIM + 1] *= sqrt_w;

        // Scale J_cameras (2x9 = 18 entries)
        for (int j = 0; j < RES_DIM * CAM_DIM; ++j) {
          J_cameras[k * RES_DIM * CAM_DIM + j] *= sqrt_w;
        }

        // Scale J_points (2x3 = 6 entries)
        for (int j = 0; j < RES_DIM * PT_DIM; ++j) {
          J_points[k * RES_DIM * PT_DIM + j] *= sqrt_w;
        }
      }
    }

    // ---- Step 2: Accumulate normal equations ----
    NormalEquations neq;
    accumulateNormalEquations(problem, residuals.data(), J_cameras.data(),
                              J_points.data(), neq);

    // ---- Check gradient convergence ----
    double max_gradient = 0.0;
    for (int i = 0; i < num_cameras; ++i) {
      max_gradient =
          std::max(max_gradient, neq.g_cameras[i].lpNorm<Eigen::Infinity>());
    }
    for (int j = 0; j < num_points; ++j) {
      max_gradient =
          std::max(max_gradient, neq.g_points[j].lpNorm<Eigen::Infinity>());
    }

    if (max_gradient < config.gradient_tolerance) {
      result.converged = true;
      result.termination_reason = "Gradient below tolerance";
      break;
    }

    // ---- Step 3: Form Schur complement ----
    Eigen::MatrixXd S;
    Eigen::VectorXd rhs;
    computeSchurComplement(neq, S, rhs);

    // ---- Inner loop: try different λ values ----
    bool step_accepted = false;

    // Save current parameters
    for (int i = 0; i < num_cameras; ++i) {
      const double* cam = problem.cameras[i]->data();
      for (int d = 0; d < CAM_DIM; ++d) {
        saved_cameras[i * CAM_DIM + d] = cam[d];
      }
    }
    for (int j = 0; j < num_points; ++j) {
      const double* pt = problem.points[j]->data();
      for (int d = 0; d < PT_DIM; ++d) {
        saved_points[j * PT_DIM + d] = pt[d];
      }
    }

    for (int retry = 0; retry < 10; ++retry) {
      // ---- Step 4: Add LM damping ----
      // (S + λ · diag(S)) · δc = -rhs
      Eigen::MatrixXd S_damped = S;
      for (int d = 0; d < S.rows(); ++d) {
        S_damped(d, d) += lambda * std::max(S(d, d), 1e-6);
      }

      // ---- Step 5: Solve reduced system ----
      Eigen::VectorXd delta_cameras = S_damped.ldlt().solve(-rhs);

      // ---- Step 6: Back-substitute for point updates ----
      Eigen::VectorXd delta_points;
      backSubstitute(neq, delta_cameras, delta_points);

      // ---- Check step size convergence ----
      double step_norm = delta_cameras.norm() + delta_points.norm();
      double param_norm = 0.0;
      for (int i = 0; i < num_cameras; ++i) {
        for (int d = 0; d < CAM_DIM; ++d) {
          param_norm +=
              saved_cameras[i * CAM_DIM + d] * saved_cameras[i * CAM_DIM + d];
        }
      }
      for (int j = 0; j < num_points; ++j) {
        for (int d = 0; d < PT_DIM; ++d) {
          param_norm +=
              saved_points[j * PT_DIM + d] * saved_points[j * PT_DIM + d];
        }
      }
      param_norm = std::sqrt(param_norm);

      if (step_norm / (param_norm + 1e-12) < config.step_tolerance) {
        result.converged = true;
        result.termination_reason = "Step size below tolerance";
        break;
      }

      // ---- Step 7: Apply update and evaluate new cost ----
      applyUpdate(problem, delta_cameras, delta_points);
      double new_cost =
          computeCost(problem, config.loss_type, config.loss_param);

      if (new_cost < current_cost) {
        // Accept step, decrease λ
        double cost_change = current_cost - new_cost;

        if (config.verbose) {
          std::cout << "LM iter " << iter + 1 << ": cost " << current_cost
                    << " -> " << new_cost << " (Δ=" << cost_change
                    << ", RMS=" << std::sqrt(2.0 * new_cost / num_obs)
                    << ", λ=" << lambda << ")" << std::endl;
        }

        current_cost = new_cost;
        lambda = std::max(lambda / config.lambda_factor, config.min_lambda);
        step_accepted = true;

        // Check cost convergence
        if (cost_change / (current_cost + 1e-12) < config.cost_tolerance) {
          result.converged = true;
          result.termination_reason = "Cost change below tolerance";
        }
        break;
      } else {
        // Reject step, restore parameters, increase λ
        for (int i = 0; i < num_cameras; ++i) {
          double* cam = problem.cameras[i]->mutable_data();
          for (int d = 0; d < CAM_DIM; ++d) {
            cam[d] = saved_cameras[i * CAM_DIM + d];
          }
        }
        for (int j = 0; j < num_points; ++j) {
          double* pt = problem.points[j]->mutable_data();
          for (int d = 0; d < PT_DIM; ++d) {
            pt[d] = saved_points[j * PT_DIM + d];
          }
        }

        lambda = std::min(lambda * config.lambda_factor, config.max_lambda);

        if (lambda >= config.max_lambda) {
          result.termination_reason = "Lambda exceeded maximum";
          break;
        }
      }
    }

    if (result.converged) break;

    if (!step_accepted) {
      result.termination_reason = "Failed to find a descent step";
      break;
    }
  }

  if (!result.converged && result.iterations >= config.max_iterations) {
    result.termination_reason = "Maximum iterations reached";
  }

  result.final_cost = current_cost;

  if (config.verbose) {
    std::cout << "LM: " << result.termination_reason << " after "
              << result.iterations << " iterations"
              << ". Final cost = " << result.final_cost
              << " (RMS = " << std::sqrt(2.0 * result.final_cost / num_obs)
              << ")" << std::endl;
  }

  return result;
}

}  // namespace solver
}  // namespace backend
}  // namespace substral
