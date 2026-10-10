// Copyright 2026 Zach. MIT License.
//
// Pure cost math for RobotAvoidanceLayer, kept free of ROS so it can be
// unit tested on its own (test/test_avoidance_cost.cpp).

#ifndef VISION_BOT_PLUGINS__AVOIDANCE_COST_HPP_
#define VISION_BOT_PLUGINS__AVOIDANCE_COST_HPP_

namespace vision_bot_plugins
{

/// Cost to write at a cell distance_m from another robot's centre.
/// 254 (lethal) within robot_radius_m; beyond it,
/// floor(252 * exp(-cost_scaling * (distance_m - robot_radius_m))). Starting
/// the decay at 252 keeps it below 253 ("inscribed"), so only the other
/// robot's own disk counts as a certain collision.
unsigned char computeAvoidanceCost(
  double distance_m, double robot_radius_m, double cost_scaling);

/// Distance from the other robot's centre beyond which computeAvoidanceCost
/// is 0: robot_radius_m + ln(253) / cost_scaling. Used for update bounds.
double avoidanceInfluenceRadius(double robot_radius_m, double cost_scaling);

/// True when a pose stamped pose_stamp_s is at most max_age_s old at now_s.
/// A robot that stops publishing (crashed, offline) stops blocking paths.
bool isPoseFresh(double pose_stamp_s, double now_s, double max_age_s);

}  // namespace vision_bot_plugins

#endif  // VISION_BOT_PLUGINS__AVOIDANCE_COST_HPP_
