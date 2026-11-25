#ifndef A6_Y_ROS2_CONTROL__STEADY_COMMAND_WATCHDOG_HPP_
#define A6_Y_ROS2_CONTROL__STEADY_COMMAND_WATCHDOG_HPP_

#include <algorithm>
#include <chrono>

namespace a6_y_ros2_control
{
class SteadyCommandWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  void reset()
  {
    initialized_ = false;
  }

  double observe()
  {
    return observe_at(Clock::now());
  }

  double observe_at(Clock::time_point now)
  {
    if (!initialized_)
    {
      initialized_ = true;
      last_observation_ = now;
      return 0.0;
    }

    const double elapsed = std::chrono::duration<double>(now - last_observation_).count();
    last_observation_ = now;
    return std::max(0.0, elapsed);
  }

private:
  bool initialized_{ false };
  Clock::time_point last_observation_{};
};
}  // namespace a6_y_ros2_control

#endif  // A6_Y_ROS2_CONTROL__STEADY_COMMAND_WATCHDOG_HPP_
