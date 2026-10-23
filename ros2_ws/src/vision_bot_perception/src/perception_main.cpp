// The standalone perception_node executable. A node that cannot start (a DNN
// model that cannot be loaded) exits with status 1 after logging why, rather
// than aborting on the uncaught exception, which would leave a crash dump.
#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "vision_bot_perception/perception_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;
  try {
    node = vision_bot_perception::make_perception_node(rclcpp::NodeOptions());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception_node"), "not starting: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
