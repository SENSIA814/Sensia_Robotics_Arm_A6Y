#ifndef A6_Y_CONTROLLERS__SMOOTH_FRICTION_COMPENSATION_HPP_
#define A6_Y_CONTROLLERS__SMOOTH_FRICTION_COMPENSATION_HPP_

#include <algorithm>
#include <cmath>

namespace a6_y_controllers
{
// Velocity-based sliding-friction assistance in joint coordinates. This cannot
// distinguish hand motion from gravity-driven motion, or infer intent at rest.
class SmoothFrictionCompensation
{
public:
  void reset() { effort_ = 0.0; }

  double update(double velocity, double period_seconds, double amplitude,
                double deadband, double velocity_scale, double ramp_duration)
  {
    // A late/invalid cycle must not produce a large accumulated ramp step.
    if (!std::isfinite(velocity) || !std::isfinite(period_seconds) ||
        period_seconds <= 0.0 || period_seconds > 0.1 ||
        !std::isfinite(amplitude) || amplitude <= 0.0 ||
        !std::isfinite(deadband) || deadband <= 0.0 ||
        !std::isfinite(velocity_scale) || velocity_scale <= 0.0 ||
        !std::isfinite(ramp_duration) || ramp_duration <= 0.0 ||
        std::abs(velocity) <= deadband)
    {
      reset();
      return 0.0;
    }

    // Drop the old direction immediately; start the reverse ramp next cycle.
    if (effort_ != 0.0 && std::signbit(effort_) != std::signbit(velocity))
    {
      reset();
      return 0.0;
    }

    const double x = std::clamp((std::abs(velocity) - deadband) / velocity_scale, 0.0, 1.0);
    const double target = amplitude * x * x * (3.0 - 2.0 * x);
    // Limit increases only: deceleration/stop must not retain excess assistance.
    const double step = amplitude * std::min(period_seconds / ramp_duration, 1.0);
    effort_ = std::copysign(std::min(target, std::abs(effort_) + step), velocity);
    return effort_;
  }

private:
  double effort_{0.0};
};
}  // namespace a6_y_controllers

#endif  // A6_Y_CONTROLLERS__SMOOTH_FRICTION_COMPENSATION_HPP_
