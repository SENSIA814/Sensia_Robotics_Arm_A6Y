#include <a6_y_controllers/gravity_home_brake.hpp>
#include <gtest/gtest.h>
#include <limits>
#include <random>

using namespace a6_y_controllers;

TEST(GravityHomeBrake, LeavesSlowCorrectionUnchangedWithinSharedBounds)
{
  for (double velocity : {-0.15, -0.1, 0.0, 0.1, 0.15})
  {
    const auto result = gravity_home_brake(0.3, velocity, -0.6, 0.6);
    ASSERT_TRUE(result.valid);
    EXPECT_FALSE(result.braking);
    EXPECT_DOUBLE_EQ(result.torque, 0.3);
  }
  EXPECT_DOUBLE_EQ(gravity_home_brake(1.0, 0.0, -0.2, 0.4).torque, 0.4);
  EXPECT_DOUBLE_EQ(gravity_home_brake(-1.0, 0.0, -0.2, 0.4).torque, -0.2);
}

TEST(GravityHomeBrake, ReplacesForwardPushWithImmediateOpposingTorque)
{
  for (double sign : {-1.0, 1.0})
  {
    const auto result = gravity_home_brake(sign * 0.4, sign * 0.25, -0.6, 0.6);
    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.braking);
    EXPECT_NEAR(result.torque, -sign * 0.03, 1e-12);
  }
}

TEST(GravityHomeBrake, PreservesAndStrengthensExistingOpposingTorque)
{
  for (double sign : {-1.0, 1.0})
  {
    const auto result = gravity_home_brake(-sign * 0.35, sign * 0.25, -0.6, 0.6);
    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.braking);
    EXPECT_NEAR(result.torque, -sign * 0.38, 1e-12);
    const auto saturated = gravity_home_brake(-sign * 0.59, sign * 0.25, -0.6, 0.6);
    EXPECT_DOUBLE_EQ(saturated.torque, -sign * 0.6);
  }
}

TEST(GravityHomeBrake, RespectsAsymmetricRemainingTotalTorqueRoom)
{
  EXPECT_DOUBLE_EQ(gravity_home_brake(0.5, 0.6, -0.1, 0.6).torque, -0.1);
  EXPECT_DOUBLE_EQ(gravity_home_brake(-0.5, -0.6, -0.6, 0.1).torque, 0.1);
  EXPECT_DOUBLE_EQ(gravity_home_brake(0.5, 0.4, 0.0, 0.6).torque, 0.0);
  EXPECT_DOUBLE_EQ(gravity_home_brake(-0.5, -0.4, -0.6, 0.0).torque, 0.0);
  EXPECT_DOUBLE_EQ(gravity_home_brake(0.5, 0.4, 0.0, 0.0).torque, 0.0);
}

TEST(GravityHomeBrake, ActivatesStrictlyAboveThresholdWithoutRampDelay)
{
  const auto result = gravity_home_brake(0.5, std::nextafter(0.15, 1.0), -0.6, 0.6);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.braking);
  EXPECT_LT(result.torque, 0.0);
  // Even an accelerating full-budget input is removed on the first sample.
  const auto first_sample = gravity_home_brake(0.6, 0.35, -0.6, 0.6);
  EXPECT_NEAR(first_sample.torque, -0.06, 1e-12);
}

TEST(GravityHomeBrake, RandomBrakingNeverAddsMeasuredJointPowerOrExceedsBudget)
{
  std::mt19937 rng(20261125);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  for (int i = 0; i < 10000; ++i)
  {
    const double velocity = (i % 2 ? -1.0 : 1.0) * (0.150001 + std::abs(uniform(rng)));
    const double correction = 2.0 * uniform(rng);
    const double lower = i % 7 ? -std::abs(uniform(rng)) : 0.0;
    const double upper = i % 11 ? std::abs(uniform(rng)) : 0.0;
    const auto result = gravity_home_brake(correction, velocity, lower, upper);
    ASSERT_TRUE(result.valid) << i;
    ASSERT_TRUE(result.braking) << i;
    EXPECT_GE(result.torque, lower) << i;
    EXPECT_LE(result.torque, upper) << i;
    EXPECT_LE(result.torque * velocity, 0.0) << i;
    if (correction * velocity < 0.0) {
      EXPECT_GE(std::abs(result.torque), std::abs(std::clamp(correction, lower, upper))) << i;
    }
  }
}

TEST(GravityHomeBrake, RejectsNonfiniteOrNonZeroContainingBounds)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto result : {
      gravity_home_brake(nan, 0.3, -0.6, 0.6),
      gravity_home_brake(0.0, inf, -0.6, 0.6),
      gravity_home_brake(0.0, 0.3, -inf, 0.6),
      gravity_home_brake(0.0, 0.3, -0.6, nan),
      gravity_home_brake(0.0, 0.3, 0.1, 0.6),
      gravity_home_brake(0.0, 0.3, -0.6, -0.1),
      gravity_home_brake(0.0, 0.3, 0.6, -0.6)})
  {
    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.braking);
    EXPECT_DOUBLE_EQ(result.torque, 0.0);
  }
}
