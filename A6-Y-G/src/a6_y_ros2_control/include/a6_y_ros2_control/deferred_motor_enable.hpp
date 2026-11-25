#ifndef A6_Y_ROS2_CONTROL__DEFERRED_MOTOR_ENABLE_HPP_
#define A6_Y_ROS2_CONTROL__DEFERRED_MOTOR_ENABLE_HPP_

#include <cmath>

namespace a6_y_ros2_control
{
inline constexpr char kMotorEnableInterfaceName[] = "motor_enable";

enum class ControllerEnableBatchState
{
  no_commandable_motor,
  waiting,
  ready,
};

enum class DeferredMotorEnableAction
{
  legacy,
  wait_disabled,
  enable_and_send,
  send,
  disable,
};

template<typename Iterator, typename ReadOnlyFn, typename EnableCommandFn>
ControllerEnableBatchState controller_enable_batch_state(
    Iterator begin, Iterator end, ReadOnlyFn is_read_only,
    EnableCommandFn enable_command_for)
{
  bool has_commandable_motor = false;
  for (auto it = begin; it != end; ++it)
  {
    if (is_read_only(*it))
    {
      continue;
    }
    has_commandable_motor = true;
    const double command = enable_command_for(*it);
    if (!std::isfinite(command) || command < 0.5)
    {
      return ControllerEnableBatchState::waiting;
    }
  }
  return has_commandable_motor ? ControllerEnableBatchState::ready :
                                 ControllerEnableBatchState::no_commandable_motor;
}

constexpr DeferredMotorEnableAction deferred_motor_enable_action(
    bool deferred_enable, ControllerEnableBatchState controller_state,
    bool any_motor_enabled)
{
  if (!deferred_enable)
  {
    return DeferredMotorEnableAction::legacy;
  }
  if (controller_state != ControllerEnableBatchState::ready)
  {
    return any_motor_enabled ? DeferredMotorEnableAction::disable :
                               DeferredMotorEnableAction::wait_disabled;
  }
  return any_motor_enabled ? DeferredMotorEnableAction::send :
                             DeferredMotorEnableAction::enable_and_send;
}
}  // namespace a6_y_ros2_control

#endif  // A6_Y_ROS2_CONTROL__DEFERRED_MOTOR_ENABLE_HPP_
