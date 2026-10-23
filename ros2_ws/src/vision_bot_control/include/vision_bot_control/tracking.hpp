// The control law behind motor_control_node, ported from
// compute_tracking_cmd in vision_bot/motor_control_node.py. No ROS here:
// test/test_tracking.cpp checks it against the Python function's outputs.
#pragma once

namespace vision_bot_control
{

struct TrackingParams
{
  double angular_gain = 1.0;        // rad/s per unit offset
  double base_linear_speed = 0.15;  // m/s
  double offset_deadband = 0.05;    // offsets smaller than this count as centered
  double stop_area_fraction = 0.15; // stop driving forward once the target is this big
};

struct TrackingCmd
{
  double linear_x = 0.0;
  double angular_z = 0.0;
};

// Detection geometry -> velocity command. Turns toward the target
// (angular_z = -angular_gain * offset_x, so a target on the right turns
// right), slows down while off-center and as the target grows, and holds
// linear_x at 0 once area_fraction reaches stop_area_fraction (still turning
// to stay centered).
TrackingCmd compute_tracking_cmd(double offset_x, double area_fraction, const TrackingParams & p);

}  // namespace vision_bot_control
