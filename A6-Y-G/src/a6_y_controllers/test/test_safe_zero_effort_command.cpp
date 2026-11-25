#include <a6_y_controllers/controller_motor_enable.hpp>
#include <a6_y_controllers/safe_zero_effort_command.hpp>

#include <gtest/gtest.h>

#include <limits>

namespace a6_y_controllers
{
TEST(ControllerMotorEnable, GravityWaitsForFullSupportCommand)
{
  EXPECT_DOUBLE_EQ(gravity_motor_enable_command("", true, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(
    gravity_motor_enable_command("motor_enable", true, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(
    gravity_motor_enable_command("motor_enable", true, 0.999), 0.0);
  EXPECT_DOUBLE_EQ(
    gravity_motor_enable_command("motor_enable", true, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(
    gravity_motor_enable_command("motor_enable", false, 1.0), 0.0);
}

TEST(ControllerMotorEnable, FollowerRequiresCompleteSafeHold)
{
  EXPECT_DOUBLE_EQ(
    safe_command_motor_enable_command("motor_enable", false), 0.0);
  EXPECT_DOUBLE_EQ(
    safe_command_motor_enable_command("motor_enable", true), 1.0);
}

TEST(SafeZeroEffortCommand, UsesMeasuredPositionWhenItIsFinite)
{
  const auto command = resolve_safe_zero_effort_command(1.25, -0.5);

  EXPECT_TRUE(command.measured_position_is_finite);
  EXPECT_DOUBLE_EQ(command.position, 1.25);
  EXPECT_DOUBLE_EQ(command.velocity, 0.0);
  EXPECT_DOUBLE_EQ(command.effort, 0.0);
}

TEST(SafeZeroEffortCommand, FallsBackWithoutRetainingEffort)
{
  const auto previous_position = resolve_safe_zero_effort_command(
    std::numeric_limits<double>::quiet_NaN(), -0.5);
  EXPECT_FALSE(previous_position.measured_position_is_finite);
  EXPECT_DOUBLE_EQ(previous_position.position, -0.5);
  EXPECT_DOUBLE_EQ(previous_position.velocity, 0.0);
  EXPECT_DOUBLE_EQ(previous_position.effort, 0.0);

  const auto zero_position = resolve_safe_zero_effort_command(
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(zero_position.measured_position_is_finite);
  EXPECT_DOUBLE_EQ(zero_position.position, 0.0);
  EXPECT_DOUBLE_EQ(zero_position.velocity, 0.0);
  EXPECT_DOUBLE_EQ(zero_position.effort, 0.0);
}
}  // namespace a6_y_controllers
