#include <a6_y_controllers/smooth_friction_compensation.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{
using a6_y_controllers::SmoothFrictionCompensation;

double step(SmoothFrictionCompensation& friction, double velocity, double dt = 0.02)
{
  return friction.update(velocity, dt, 0.15, 0.02, 0.05, 0.2);
}

TEST(SmoothFrictionCompensationTest, StationaryFeedbackNoiseDoesNotStartAssistance)
{
  SmoothFrictionCompensation friction;
  for (int i = 0; i < 100; ++i)
    for (double velocity : {0.0, 0.002442, -0.007326, 0.02, -0.02})
      EXPECT_DOUBLE_EQ(step(friction, velocity), 0.0);
}

TEST(SmoothFrictionCompensationTest, SustainsAssistanceBeyondLegacyPulseDuration)
{
  for (double direction : {-1.0, 1.0})
  {
    SmoothFrictionCompensation friction;
    double previous = 0.0;
    for (int i = 0; i < 250; ++i)
    {
      const double effort = step(friction, direction * 0.1);
      EXPECT_GE(effort * direction, 0.0);
      EXPECT_LE(std::abs(effort), 0.15);
      EXPECT_LE(std::abs(effort - previous), 0.015 + 1e-12);
      if (i >= 9) { EXPECT_NEAR(effort, direction * 0.15, 1e-12); }
      previous = effort;
    }
  }
}

TEST(SmoothFrictionCompensationTest, SlowingStoppingAndReversingDiscardOldAssistance)
{
  SmoothFrictionCompensation friction;
  for (int i = 0; i < 10; ++i) step(friction, 0.1);
  const double slow_effort = step(friction, 0.021);
  EXPECT_GT(slow_effort, 0.0);
  EXPECT_LT(slow_effort, 0.001);
  EXPECT_DOUBLE_EQ(step(friction, 0.0), 0.0);
  EXPECT_NEAR(step(friction, 0.1), 0.015, 1e-12);
  EXPECT_DOUBLE_EQ(step(friction, -0.1), 0.0);
  EXPECT_NEAR(step(friction, -0.1), -0.015, 1e-12);
  EXPECT_DOUBLE_EQ(step(friction, 0.1), 0.0);
}

TEST(SmoothFrictionCompensationTest, RampUsesElapsedTimeAndResetDropsHistory)
{
  SmoothFrictionCompensation fast, slow;
  double fast_effort = 0.0, slow_effort = 0.0;
  for (int i = 0; i < 10; ++i) fast_effort = step(fast, 0.1, 0.01);
  for (int i = 0; i < 5; ++i) slow_effort = step(slow, 0.1, 0.02);
  EXPECT_NEAR(fast_effort, slow_effort, 1e-12);
  EXPECT_NEAR(fast_effort, 0.075, 1e-12);
  fast.reset();
  EXPECT_NEAR(step(fast, 0.1), 0.015, 1e-12);
}

TEST(SmoothFrictionCompensationTest, InvalidOrLateSamplesClearState)
{
  SmoothFrictionCompensation friction;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (double dt : {0.0, -0.02, 0.101, nan, inf})
  {
    step(friction, 0.1);
    EXPECT_DOUBLE_EQ(step(friction, 0.1, dt), 0.0);
    EXPECT_NEAR(step(friction, 0.1), 0.015, 1e-12);
  }
  for (double velocity : {nan, inf, -inf})
  {
    EXPECT_DOUBLE_EQ(step(friction, velocity), 0.0);
    EXPECT_NEAR(step(friction, 0.1), 0.015, 1e-12);
  }
  EXPECT_DOUBLE_EQ(friction.update(0.1, 0.02, 0, 0.02, 0.05, 0.2), 0.0);
  EXPECT_DOUBLE_EQ(friction.update(0.1, 0.02, -0.15, 0.02, 0.05, 0.2), 0.0);
  EXPECT_DOUBLE_EQ(friction.update(0.1, 0.02, 0.15, 0, 0.05, 0.2), 0.0);
  EXPECT_DOUBLE_EQ(friction.update(0.1, 0.02, 0.15, 0.02, 0, 0.2), 0.0);
  EXPECT_DOUBLE_EQ(friction.update(0.1, 0.02, 0.15, 0.02, 0.05, 0), 0.0);
}

TEST(SmoothFrictionCompensationTest, TransitionIsBoundedMonotonicAndSymmetric)
{
  double previous = 0.0;
  for (int i = 0; i <= 100; ++i)
  {
    SmoothFrictionCompensation positive, negative;
    double forward = 0.0, backward = 0.0;
    const double speed = i * 0.001;
    for (int cycle = 0; cycle < 10; ++cycle)
    {
      forward = step(positive, speed);
      backward = step(negative, -speed);
    }
    EXPECT_GE(forward, previous);
    EXPECT_LE(forward, 0.15);
    EXPECT_DOUBLE_EQ(forward, -backward);
    if (speed <= 0.02) { EXPECT_DOUBLE_EQ(forward, 0.0); }
    previous = forward;
  }
}
}  // namespace
