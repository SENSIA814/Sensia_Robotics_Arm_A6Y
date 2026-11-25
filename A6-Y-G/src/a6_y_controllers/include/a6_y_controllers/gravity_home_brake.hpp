#pragma once

#include <algorithm>
#include <cmath>

namespace a6_y_controllers
{
struct GravityHomeBrakeResult
{
  bool valid{false};
  bool braking{false};
  double torque{0.0};
};

// Apply to the final incremental torque (including any integral), after the
// impedance solve. Bounds are the shared incremental and total-torque room
// remaining after gravity, and must include zero. Do not slew-limit the result:
// doing so could retain an accelerating torque while braking is required.
//
// This is a dissipative soft brake, not a replacement for raw-feedback speed
// protection. The caller retains its hard limit and clears any integral that
// pushes in the measured motion direction while `braking` is true. A sampled
// torque opposing measured velocity does not itself prove delayed-plant safety.
inline GravityHomeBrakeResult gravity_home_brake(
    double correction, double velocity, double lower, double upper)
{
  GravityHomeBrakeResult result;
  if (!std::isfinite(correction) || !std::isfinite(velocity) ||
      !std::isfinite(lower) || !std::isfinite(upper) ||
      lower > 0.0 || upper < 0.0 || lower > upper)
    return result;

  constexpr double soft_speed = 0.15;  // rad/s; hard-speed check is external.
  constexpr double brake_gain = 0.3;   // Nm/(rad/s), supplementary J4 damping.
  result.valid = true;
  result.braking = std::abs(velocity) > soft_speed;
  double torque = correction;
  if (result.braking)
  {
    // Retain stronger existing braking, remove any accelerating correction,
    // then add direct damping without the model-inertia implicit attenuation.
    const double brake = brake_gain * (std::abs(velocity) - soft_speed);
    torque = velocity > 0.0 ? std::min(correction, 0.0) - brake :
                             std::max(correction, 0.0) + brake;
  }
  result.torque = std::clamp(torque, lower, upper);
  return result;
}
}  // namespace a6_y_controllers
