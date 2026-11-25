#include <a6_y_controllers/haptic_effort_composer.hpp>
#include <a6_y_controllers/staged_output_gate.hpp>

#include <gtest/gtest.h>

#include <limits>

namespace a6_y_controllers
{
TEST(HapticCommandFreshness, RequiresPositiveTimeoutExactDimensionAndPastTimestamp)
{
  EXPECT_TRUE(haptic_command_is_fresh(10.05, 10.0, 0.1, 6, 6));
  EXPECT_FALSE(haptic_command_is_fresh(10.11, 10.0, 0.1, 6, 6));
  EXPECT_FALSE(haptic_command_is_fresh(10.05, 10.0, 0.0, 6, 6));
  EXPECT_FALSE(haptic_command_is_fresh(10.05, 10.0, 0.1, 5, 6));
  EXPECT_FALSE(haptic_command_is_fresh(10.0, 10.05, 0.1, 6, 6));
}

TEST(HapticActivationRamp, StaleCommandResetsRampBeforeFreshCommandResumes)
{
  OutputActivationRamp ramp;
  constexpr double ramp_duration_seconds = 2.0;

  EXPECT_DOUBLE_EQ(
    ramp.update(
      staged_output_is_ready(true, true, true),
      10.0, ramp_duration_seconds),
    0.0);
  EXPECT_DOUBLE_EQ(
    ramp.update(
      staged_output_is_ready(true, true, true),
      10.5, ramp_duration_seconds),
    0.25);

  EXPECT_DOUBLE_EQ(
    ramp.update(
      staged_output_is_ready(true, true, false),
      10.7, ramp_duration_seconds),
    0.0);

  EXPECT_DOUBLE_EQ(
    ramp.update(
      staged_output_is_ready(true, true, true),
      11.0, ramp_duration_seconds),
    0.0);
  EXPECT_DOUBLE_EQ(
    ramp.update(
      staged_output_is_ready(true, true, true),
      11.5, ramp_duration_seconds),
    0.25);
}

TEST(HapticEffortComposer, DisabledFeedbackPreservesBaseEffort)
{
  const auto result = compose_haptic_effort(
    1.25,
    -0.8,
    0.5,
    2.0,
    false,
    true);

  EXPECT_DOUBLE_EQ(result.applied_haptic_effort, 0.0);
  EXPECT_DOUBLE_EQ(result.output_effort, 1.25);
}

TEST(HapticEffortComposer, AppliesIndependentHapticAndTotalEffortLimits)
{
  const auto result = compose_haptic_effort(
    1.5, 2.0, 0.4, 1.7, true, true);

  EXPECT_DOUBLE_EQ(result.applied_haptic_effort, 0.4);
  EXPECT_DOUBLE_EQ(result.requested_total_effort, 1.9);
  EXPECT_DOUBLE_EQ(result.output_effort, 1.7);
}

TEST(HapticEffortComposer, StaleOrInvalidHapticCommandPreservesBaseEffort)
{
  const auto stale = compose_haptic_effort(
    0.75, 0.5, 0.6, 2.0, true, false);
  EXPECT_DOUBLE_EQ(stale.applied_haptic_effort, 0.0);
  EXPECT_DOUBLE_EQ(stale.output_effort, 0.75);

  const auto invalid = compose_haptic_effort(
    0.75, std::numeric_limits<double>::quiet_NaN(), 0.6, 2.0, true, true);
  EXPECT_DOUBLE_EQ(invalid.applied_haptic_effort, 0.0);
  EXPECT_DOUBLE_EQ(invalid.output_effort, 0.75);
}

TEST(HapticEffortComposer, InvalidBaseEffortFailsClosed)
{
  const auto result = compose_haptic_effort(
    std::numeric_limits<double>::infinity(), 0.5, 0.6, 2.0, true, true);
  EXPECT_DOUBLE_EQ(result.applied_haptic_effort, 0.0);
  EXPECT_DOUBLE_EQ(result.requested_total_effort, 0.0);
  EXPECT_DOUBLE_EQ(result.output_effort, 0.0);
}
}  // namespace a6_y_controllers
