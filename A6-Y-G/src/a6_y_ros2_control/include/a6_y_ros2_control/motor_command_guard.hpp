#ifndef A6_Y_ROS2_CONTROL__MOTOR_COMMAND_GUARD_HPP_
#define A6_Y_ROS2_CONTROL__MOTOR_COMMAND_GUARD_HPP_

namespace a6_y_ros2_control
{
enum class MotorCommandDisposition
{
  skip_read_only,
  send,
  block_disabled,
  block_faulted,
};

constexpr MotorCommandDisposition motor_command_disposition(bool read_only, bool enabled, bool faulted)
{
  if (read_only)
  {
    return MotorCommandDisposition::skip_read_only;
  }
  if (!enabled)
  {
    return MotorCommandDisposition::block_disabled;
  }
  if (faulted)
  {
    return MotorCommandDisposition::block_faulted;
  }
  return MotorCommandDisposition::send;
}

template<typename Iterator, typename DispositionFn>
constexpr MotorCommandDisposition motor_command_batch_disposition(
    Iterator begin, Iterator end, DispositionFn disposition_for)
{
  bool has_commandable_motor = false;
  for (auto it = begin; it != end; ++it)
  {
    const auto disposition = disposition_for(*it);
    if (disposition == MotorCommandDisposition::block_disabled ||
        disposition == MotorCommandDisposition::block_faulted)
    {
      return disposition;
    }
    has_commandable_motor = has_commandable_motor || disposition == MotorCommandDisposition::send;
  }
  return has_commandable_motor ? MotorCommandDisposition::send : MotorCommandDisposition::skip_read_only;
}
}  // namespace a6_y_ros2_control

#endif  // A6_Y_ROS2_CONTROL__MOTOR_COMMAND_GUARD_HPP_
