// The C++ control law against the Python one: test/fixtures/reference.json
// holds what vision_bot/motor_control_node.py's compute_tracking_cmd returned
// for each case (scripts/make_control_fixtures.py).
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#include "vision_bot_control/tracking.hpp"

using vision_bot_control::TrackingCmd;
using vision_bot_control::TrackingParams;
using vision_bot_control::compute_tracking_cmd;

TEST(TrackingReference, MatchesPythonOnEveryCase)
{
  std::ifstream in(std::string(FIXTURE_DIR) + "/reference.json");
  const auto cases = nlohmann::json::parse(in)["cases"];
  ASSERT_GE(cases.size(), 200u);
  for (const auto & c : cases) {
    const auto & p = c["params"];
    TrackingParams params;
    params.angular_gain = p["angular_gain"];
    params.base_linear_speed = p["base_linear_speed"];
    params.offset_deadband = p["offset_deadband"];
    params.stop_area_fraction = p["stop_area_fraction"];
    const double offset = c["offset_x"];
    const double area = c["area_fraction"];
    const TrackingCmd cmd = compute_tracking_cmd(offset, area, params);
    const double linear = c["expected"][0];
    const double angular = c["expected"][1];
    const std::string name = c.dump();
    EXPECT_NEAR(cmd.linear_x, linear, 1e-12) << name;
    EXPECT_NEAR(cmd.angular_z, angular, 1e-12) << name;
    // -gain * 0.0 is -0.0 in Python, and that reaches /cmd_vel as is.
    EXPECT_EQ(std::signbit(cmd.angular_z), std::signbit(angular)) << name;
  }
}

TEST(Tracking, NeverDividesWhenStopAreaIsZero)
{
  TrackingParams params;
  params.stop_area_fraction = 0.0;
  const TrackingCmd cmd = compute_tracking_cmd(0.3, 0.0, params);
  EXPECT_TRUE(std::isfinite(cmd.linear_x));
  EXPECT_EQ(cmd.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular_z, -0.3);
}
