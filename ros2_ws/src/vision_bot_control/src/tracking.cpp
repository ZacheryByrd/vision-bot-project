#include "vision_bot_control/tracking.hpp"

#include <algorithm>
#include <cmath>

namespace vision_bot_control
{

TrackingCmd compute_tracking_cmd(double offset_x, double area_fraction, const TrackingParams & p)
{
  if (std::abs(offset_x) < p.offset_deadband) {
    offset_x = 0.0;
  }

  TrackingCmd cmd;
  cmd.angular_z = -p.angular_gain * offset_x;

  if (area_fraction >= p.stop_area_fraction) {
    // Close enough: hold position (still turning to stay centered) instead
    // of creeping forward into the target. This branch is also taken when
    // stop_area_fraction <= 0, so the division below never sees a zero.
    cmd.linear_x = 0.0;
  } else {
    // Slow down in proportion to how far off-center the target is, and
    // further as it grows toward the stop threshold.
    const double proximity_factor = std::max(0.2, 1.0 - area_fraction / p.stop_area_fraction);
    cmd.linear_x = p.base_linear_speed * (1.0 - std::min(1.0, std::abs(offset_x))) *
      proximity_factor;
  }
  return cmd;
}

}  // namespace vision_bot_control
