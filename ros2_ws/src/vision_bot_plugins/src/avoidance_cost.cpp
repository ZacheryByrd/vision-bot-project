// Copyright 2026 Zach. MIT License.

#include "vision_bot_plugins/avoidance_cost.hpp"

#include <cmath>

namespace vision_bot_plugins
{

unsigned char computeAvoidanceCost(
  double distance_m, double robot_radius_m, double cost_scaling)
{
  if (distance_m <= robot_radius_m) {
    return 254;  // nav2_costmap_2d::LETHAL_OBSTACLE: the other robot's body
  }
  const double cost = 252.0 * std::exp(-cost_scaling * (distance_m - robot_radius_m));
  return static_cast<unsigned char>(std::floor(cost));
}

double avoidanceInfluenceRadius(double robot_radius_m, double cost_scaling)
{
  // 252 * exp(-cost_scaling * d) < 1 for d > ln(252) / cost_scaling; ln(253)
  // keeps the value strictly below 1 at the boundary despite rounding.
  return robot_radius_m + std::log(253.0) / cost_scaling;
}

bool isPoseFresh(double pose_stamp_s, double now_s, double max_age_s)
{
  return now_s - pose_stamp_s <= max_age_s;
}

}  // namespace vision_bot_plugins
