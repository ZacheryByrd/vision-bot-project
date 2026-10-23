// The C++ perception_node (src/perception_node.cpp), for perception_main.cpp.
// Containers load it as the component vision_bot_perception::PerceptionNode.
#pragma once

#include <rclcpp/rclcpp.hpp>

namespace vision_bot_perception
{

// Throws std::runtime_error if the node cannot start (a DNN model that cannot
// be loaded).
rclcpp::Node::SharedPtr make_perception_node(const rclcpp::NodeOptions & options);

}  // namespace vision_bot_perception
