#include "subastral/backend/solver/loss_function.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace substral::backend::solver;

// =============================================================================
// Loss Function Unit Tests
// =============================================================================
//
// Verify rho(s) and weight(s) = rho'(s) for each loss function at specific
// points, checking:
//   1. Correct values at s=0 (all should behave like L2)
//   2. Correct values in the quadratic region (Huber)
//   3. Correct values in the linear/log region (Huber/Cauchy)
//   4. Monotonicity and positivity of rho
//   5. weight(s) in [0, 1] and decreasing for robust losses
//
// =============================================================================

// ---- TrivialLoss ----

TEST(LossFunctionTest, TrivialLoss_RhoIsIdentity) {
  EXPECT_DOUBLE_EQ(TrivialLoss::rho(0.0), 0.0);
  EXPECT_DOUBLE_EQ(TrivialLoss::rho(1.0), 1.0);
  EXPECT_DOUBLE_EQ(TrivialLoss::rho(100.0), 100.0);
}

TEST(LossFunctionTest, TrivialLoss_WeightIsOne) {
  EXPECT_DOUBLE_EQ(TrivialLoss::weight(0.0), 1.0);
  EXPECT_DOUBLE_EQ(TrivialLoss::weight(1.0), 1.0);
  EXPECT_DOUBLE_EQ(TrivialLoss::weight(100.0), 1.0);
}

// ---- HuberLoss ----

TEST(LossFunctionTest, HuberLoss_QuadraticRegion) {
  // For sqrt(s) <= delta, rho(s) = s, weight = 1
  HuberLoss loss(2.0);  // delta = 2

  // s = 1.0, sqrt(s) = 1.0 <= 2.0 => quadratic
  EXPECT_DOUBLE_EQ(loss.rho(1.0), 1.0);
  EXPECT_DOUBLE_EQ(loss.weight(1.0), 1.0);

  // s = 4.0, sqrt(s) = 2.0 == delta => boundary, still quadratic
  EXPECT_DOUBLE_EQ(loss.rho(4.0), 4.0);
  EXPECT_DOUBLE_EQ(loss.weight(4.0), 1.0);
}

TEST(LossFunctionTest, HuberLoss_LinearRegion) {
  // For sqrt(s) > delta:
  //   rho(s) = 2*delta*sqrt(s) - delta^2
  //   weight = delta / sqrt(s)
  HuberLoss loss(2.0);  // delta = 2

  // s = 9.0, sqrt(s) = 3.0 > 2.0 => linear
  double expected_rho = 2.0 * 2.0 * 3.0 - 2.0 * 2.0;  // 12 - 4 = 8
  EXPECT_DOUBLE_EQ(loss.rho(9.0), expected_rho);

  double expected_weight = 2.0 / 3.0;
  EXPECT_NEAR(loss.weight(9.0), expected_weight, 1e-15);

  // s = 100.0, sqrt(s) = 10.0 => linear
  expected_rho = 2.0 * 2.0 * 10.0 - 4.0;  // 40 - 4 = 36
  EXPECT_DOUBLE_EQ(loss.rho(100.0), expected_rho);
  EXPECT_NEAR(loss.weight(100.0), 2.0 / 10.0, 1e-15);
}

TEST(LossFunctionTest, HuberLoss_Continuity) {
  // rho should be continuous at the transition s = delta^2
  HuberLoss loss(3.0);
  double s_transition = 9.0;  // delta^2

  // Approach from below
  double rho_below = loss.rho(s_transition - 1e-10);
  // Approach from above
  double rho_above = loss.rho(s_transition + 1e-10);

  EXPECT_NEAR(rho_below, rho_above, 1e-4);
  EXPECT_NEAR(loss.rho(s_transition), s_transition, 1e-15);
}

TEST(LossFunctionTest, HuberLoss_RhoLessThanS) {
  // In the linear region, rho(s) < s (outlier down-weighting)
  HuberLoss loss(1.0);
  for (double s : {4.0, 9.0, 25.0, 100.0}) {
    EXPECT_LT(loss.rho(s), s) << "rho(s) should be < s for s > delta^2";
  }
}

TEST(LossFunctionTest, HuberLoss_WeightDecreasing) {
  HuberLoss loss(1.0);
  double prev_w = loss.weight(1.0);  // = 1.0 (boundary)
  for (double s : {4.0, 9.0, 25.0, 100.0}) {
    double w = loss.weight(s);
    EXPECT_LE(w, prev_w) << "Weight should decrease with s";
    EXPECT_GT(w, 0.0) << "Weight should be positive";
    prev_w = w;
  }
}

TEST(LossFunctionTest, HuberLoss_WeightAtZero) {
  HuberLoss loss(1.0);
  EXPECT_DOUBLE_EQ(loss.weight(0.0), 1.0);
}

// ---- CauchyLoss ----

TEST(LossFunctionTest, CauchyLoss_AtZero) {
  CauchyLoss loss(2.0);
  EXPECT_NEAR(loss.rho(0.0), 0.0, 1e-15);
  EXPECT_NEAR(loss.weight(0.0), 1.0, 1e-15);
}

TEST(LossFunctionTest, CauchyLoss_KnownValues) {
  // rho(s) = c^2 * log(1 + s/c^2)
  // weight(s) = c^2 / (c^2 + s)
  CauchyLoss loss(2.0);  // c = 2, c_sq = 4

  // s = 4.0: rho = 4 * log(1 + 1) = 4 * ln(2)
  EXPECT_NEAR(loss.rho(4.0), 4.0 * std::log(2.0), 1e-15);
  // weight = 4 / (4 + 4) = 0.5
  EXPECT_NEAR(loss.weight(4.0), 0.5, 1e-15);

  // s = 0.0: rho = 4 * log(1) = 0
  EXPECT_NEAR(loss.rho(0.0), 0.0, 1e-15);
  // weight = 4 / 4 = 1
  EXPECT_NEAR(loss.weight(0.0), 1.0, 1e-15);

  // s = 12.0: rho = 4 * log(1 + 3) = 4 * ln(4)
  EXPECT_NEAR(loss.rho(12.0), 4.0 * std::log(4.0), 1e-15);
  // weight = 4 / 16 = 0.25
  EXPECT_NEAR(loss.weight(12.0), 0.25, 1e-15);
}

TEST(LossFunctionTest, CauchyLoss_RhoLessThanS) {
  // For s > 0, rho(s) < s (logarithmic growth vs linear)
  CauchyLoss loss(1.0);
  for (double s : {1.0, 4.0, 9.0, 25.0, 100.0}) {
    EXPECT_LT(loss.rho(s), s) << "rho(s) should be < s for Cauchy";
  }
}

TEST(LossFunctionTest, CauchyLoss_WeightDecreasing) {
  CauchyLoss loss(1.0);
  double prev_w = 1.0;
  for (double s : {0.1, 1.0, 4.0, 9.0, 25.0, 100.0}) {
    double w = loss.weight(s);
    EXPECT_LT(w, prev_w) << "Cauchy weight should strictly decrease";
    EXPECT_GT(w, 0.0) << "Weight should be positive";
    prev_w = w;
  }
}

TEST(LossFunctionTest, CauchyLoss_StrongerThanHuber) {
  // Cauchy should down-weight large residuals more aggressively than Huber
  HuberLoss huber(1.0);
  CauchyLoss cauchy(1.0);

  for (double s : {4.0, 9.0, 25.0, 100.0}) {
    EXPECT_LT(cauchy.weight(s), huber.weight(s))
        << "Cauchy should down-weight more than Huber at s=" << s;
  }
}

// ---- Quadratic approximation near zero ----

TEST(LossFunctionTest, AllLosses_QuadraticNearZero) {
  // Near s=0, all losses should behave like rho(s) ~ s
  HuberLoss huber(1.0);
  CauchyLoss cauchy(1.0);

  double s = 1e-10;
  EXPECT_NEAR(TrivialLoss::rho(s) / s, 1.0, 1e-6);
  EXPECT_NEAR(huber.rho(s) / s, 1.0, 1e-6);
  EXPECT_NEAR(cauchy.rho(s) / s, 1.0, 1e-6);
}
