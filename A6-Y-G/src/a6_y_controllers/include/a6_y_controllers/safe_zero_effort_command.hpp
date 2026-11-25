#ifndef A6_Y_CONTROLLERS__SAFE_ZERO_EFFORT_COMMAND_HPP_
#define A6_Y_CONTROLLERS__SAFE_ZERO_EFFORT_COMMAND_HPP_

#include <cmath>

namespace a6_y_controllers
{
struct SafeZeroEffortCommand
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  bool measured_position_is_finite{false};
};

inline SafeZeroEffortCommand resolve_safe_zero_effort_command(
    double measured_position, double previous_command_position)
{
  SafeZeroEffortCommand command;
  command.measured_position_is_finite = std::isfinite(measured_position);
  if (command.measured_position_is_finite)
  {
    command.position = measured_position;
  }
  else if (std::isfinite(previous_command_position))
  {
    command.position = previous_command_position;
  }
  return command;
}
}  // namespace a6_y_controllers

#endif  // A6_Y_CONTROLLERS__SAFE_ZERO_EFFORT_COMMAND_HPP_
