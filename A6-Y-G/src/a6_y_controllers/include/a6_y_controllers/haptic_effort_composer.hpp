#ifndef A6_Y_CONTROLLERS__HAPTIC_EFFORT_COMPOSER_HPP_
#define A6_Y_CONTROLLERS__HAPTIC_EFFORT_COMPOSER_HPP_

#include <algorithm>
#include <cstddef>
#include <cmath>

namespace a6_y_controllers
{

struct HapticEffortComposition
{
  double applied_haptic_effort{0.0};
  double requested_total_effort{0.0};
  double output_effort{0.0};
};

inline bool haptic_command_is_fresh(
  const double now_seconds,
  const double received_time_seconds,
  const double timeout_seconds,
  const std::size_t command_size,
  const std::size_t expected_size)
{
  if (!std::isfinite(now_seconds) ||
    !std::isfinite(received_time_seconds) ||
    !std::isfinite(timeout_seconds) ||
    timeout_seconds <= 0.0 ||
    command_size != expected_size ||
    now_seconds < received_time_seconds)
  {
    return false;
  }

  return now_seconds - received_time_seconds <= timeout_seconds;
}

inline HapticEffortComposition compose_haptic_effort(
  const double base_effort,
  const double requested_haptic_effort,
  const double haptic_effort_limit,
  const double total_effort_limit,
  const bool haptic_output_enabled,
  const bool haptic_command_fresh)
{
  HapticEffortComposition result;

  if (!std::isfinite(base_effort)) {
    return result;
  }

  const bool haptic_request_is_safe =
    haptic_output_enabled && haptic_command_fresh &&
    std::isfinite(requested_haptic_effort) &&
    std::isfinite(haptic_effort_limit) &&
    haptic_effort_limit > 0.0;

  if (haptic_request_is_safe) {
    result.applied_haptic_effort = std::clamp(
      requested_haptic_effort, -haptic_effort_limit, haptic_effort_limit);
  }

  result.requested_total_effort = base_effort + result.applied_haptic_effort;
  result.output_effort = result.requested_total_effort;

  if (std::isfinite(total_effort_limit) && total_effort_limit > 0.0) {
    result.output_effort = std::clamp(
      result.output_effort, -total_effort_limit, total_effort_limit);
  }

  return result;
}

}  // namespace a6_y_controllers

#endif  // A6_Y_CONTROLLERS__HAPTIC_EFFORT_COMPOSER_HPP_
