#pragma once
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace a6_y_ros2_control {
// An arriving frame is not necessarily an accepted position sample.
struct AcceptedPosition {
  double last_accepted_time{0};
  uint64_t consecutive_rejections{0};
  bool initialized{false}, valid{false};
  bool update(double raw, double now, double threshold, double alpha, double& filtered) {
// threshold == 0 disables jump rejection.
    valid = std::isfinite(raw) && std::isfinite(now) &&
      (threshold == 0.0 || !initialized ||
      std::abs(raw - filtered) <= threshold);
    if (!valid) { ++consecutive_rejections; return false; }
    filtered=initialized ? alpha*raw+(1-alpha)*filtered:raw;
    initialized=true; consecutive_rejections=0; last_accepted_time=now;
    return true;
  }
  double age(double now) const { return initialized ? std::max(0.0,now-last_accepted_time):-1; }
};
}  // namespace a6_y_ros2_control
