#ifndef A6_Y_CONTROLLERS__CONTROLLER_MOTOR_ENABLE_HPP_
#define A6_Y_CONTROLLERS__CONTROLLER_MOTOR_ENABLE_HPP_

#include <cmath>
#include <string>

namespace a6_y_controllers
{
inline double safe_command_motor_enable_command(
    const std::string& interface_name, bool safe_command_ready)
{
  return !interface_name.empty() && safe_command_ready ? 1.0 : 0.0;
}

inline double gravity_motor_enable_command(
    const std::string& interface_name, bool output_enabled,
    double base_activation_scale)
{
  return safe_command_motor_enable_command(
      interface_name,
      output_enabled && std::isfinite(base_activation_scale) &&
          base_activation_scale >= 1.0);
}
}  // namespace a6_y_controllers

#endif  // A6_Y_CONTROLLERS__CONTROLLER_MOTOR_ENABLE_HPP_
