#ifndef A6_Y_ROS2_CONTROL__STATUS_REQUEST_POLICY_HPP_
#define A6_Y_ROS2_CONTROL__STATUS_REQUEST_POLICY_HPP_

namespace a6_y_ros2_control
{
enum class StatusRequestScope
{
  none,
  read_only_joints,
  all_joints,
};

constexpr StatusRequestScope status_request_scope(bool auto_enable, bool has_read_only_joint)
{
  if (!auto_enable)
  {
    return StatusRequestScope::all_joints;
  }
  return has_read_only_joint ? StatusRequestScope::read_only_joints : StatusRequestScope::none;
}

constexpr StatusRequestScope runtime_status_request_scope(
    bool auto_enable, bool has_read_only_joint,
    bool defer_enable_until_controller_ready,
    bool all_commandable_motors_enabled)
{
  if (defer_enable_until_controller_ready &&
      !all_commandable_motors_enabled)
  {
    return StatusRequestScope::all_joints;
  }
  return status_request_scope(auto_enable, has_read_only_joint);
}

constexpr bool status_request_includes_joint(StatusRequestScope scope, bool read_only)
{
  return scope == StatusRequestScope::all_joints ||
         (scope == StatusRequestScope::read_only_joints && read_only);
}
}  // namespace a6_y_ros2_control

#endif  // A6_Y_ROS2_CONTROL__STATUS_REQUEST_POLICY_HPP_
