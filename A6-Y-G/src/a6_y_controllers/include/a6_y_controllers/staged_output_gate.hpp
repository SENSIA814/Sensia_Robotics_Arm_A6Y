#ifndef A6_Y_CONTROLLERS__STAGED_OUTPUT_GATE_HPP_
#define A6_Y_CONTROLLERS__STAGED_OUTPUT_GATE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace a6_y_controllers
{
struct TimedOutputGateCommand
{
  bool enabled{false};
  double received_time_seconds{-std::numeric_limits<double>::infinity()};
};

inline double staged_output_steady_time_seconds()
{
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

inline bool timed_output_gate_is_active(
    double now_seconds, double timeout_seconds,
    const TimedOutputGateCommand &command)
{
  return command.enabled && std::isfinite(now_seconds) &&
         std::isfinite(timeout_seconds) && timeout_seconds > 0.0 &&
         std::isfinite(command.received_time_seconds) &&
         command.received_time_seconds <= now_seconds &&
         now_seconds - command.received_time_seconds <= timeout_seconds;
}

inline bool staged_output_is_ready(
    bool output_enabled, bool gate_active, bool command_fresh)
{
  return output_enabled && gate_active && command_fresh;
}

class OutputActivationRamp
{
public:
  void reset()
  {
    activation_start_seconds_ = -std::numeric_limits<double>::infinity();
    last_update_seconds_ = -std::numeric_limits<double>::infinity();
  }

  double update(bool gate_active, double now_seconds,
                double ramp_duration_seconds)
  {
    if (!gate_active || !std::isfinite(now_seconds) ||
        !std::isfinite(ramp_duration_seconds) ||
        ramp_duration_seconds <= 0.0)
    {
      reset();
      return 0.0;
    }
    if (!std::isfinite(activation_start_seconds_) ||
        (std::isfinite(last_update_seconds_) &&
         now_seconds < last_update_seconds_))
    {
      activation_start_seconds_ = now_seconds;
    }
    last_update_seconds_ = now_seconds;
    return std::clamp(
        (now_seconds - activation_start_seconds_) / ramp_duration_seconds,
        0.0, 1.0);
  }

private:
  double activation_start_seconds_{
      -std::numeric_limits<double>::infinity()};
  double last_update_seconds_{-std::numeric_limits<double>::infinity()};
};
} // namespace a6_y_controllers

#endif // A6_Y_CONTROLLERS__STAGED_OUTPUT_GATE_HPP_
