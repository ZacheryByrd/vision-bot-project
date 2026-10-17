// Copyright 2026 Zach. MIT License.
//
// RobotAvoidanceLayer: marks the other robots' shared poses in a Nav2 costmap.
//
// A vision_bot cannot see another vision_bot with its 2D lidar: the scan
// plane is 0.083 m above the floor, the chassis top is at 0.063 m, and the
// lidar housing has no collision shape. The obstacle layer therefore never
// marks another robot. This layer subscribes to each other robot's pose
// topic and draws a lethal disk of robot_radius around it, with a cost that
// decays beyond (computeAvoidanceCost). Poses older than max_pose_age_s are
// skipped, so a robot that goes offline stops blocking paths.

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vision_bot_plugins/avoidance_cost.hpp"

namespace vision_bot_plugins
{

class RobotAvoidanceLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("RobotAvoidanceLayer: unable to lock node");
    }

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("robot_pose_topics", rclcpp::ParameterValue(std::vector<std::string>{}));
    declareParameter("robot_radius", rclcpp::ParameterValue(0.15));
    declareParameter("cost_scaling", rclcpp::ParameterValue(3.0));
    declareParameter("max_pose_age_s", rclcpp::ParameterValue(2.0));

    std::vector<std::string> topics;
    node->get_parameter(getFullName("enabled"), enabled_);
    node->get_parameter(getFullName("robot_pose_topics"), topics);
    node->get_parameter(getFullName("robot_radius"), robot_radius_);
    node->get_parameter(getFullName("cost_scaling"), cost_scaling_);
    node->get_parameter(getFullName("max_pose_age_s"), max_pose_age_s_);

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    for (const auto & topic : topics) {
      subscriptions_.push_back(
        node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic, rclcpp::QoS(10),
          [this, topic](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_[topic] = *msg;
          },
          options));
    }

    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&RobotAvoidanceLayer::onSetParameters, this, std::placeholders::_1));

    current_ = true;
    RCLCPP_INFO(
      logger_, "RobotAvoidanceLayer '%s': %zu pose topic(s), robot_radius %.2f m, "
      "cost_scaling %.2f, max_pose_age_s %.1f, %s", name_.c_str(), topics.size(),
      robot_radius_, cost_scaling_, max_pose_age_s_, enabled_ ? "enabled" : "disabled");
  }

  void updateBounds(
    double, double, double, double * min_x, double * min_y, double * max_x,
    double * max_y) override
  {
    // The layered costmap resets the master grid only inside the bounds the
    // layers report, so report last cycle's area too: that erases a robot's
    // old position after it moves, or after this layer is disabled.
    if (have_last_bounds_) {
      expand(last_bounds_, min_x, min_y, max_x, max_y);
      have_last_bounds_ = false;
    }

    active_.clear();
    double radius, scaling, max_age;
    std::map<std::string, geometry_msgs::msg::PoseWithCovarianceStamped> latest;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!enabled_) {
        return;
      }
      radius = robot_radius_;
      scaling = cost_scaling_;
      max_age = max_pose_age_s_;
      latest = latest_;
    }

    const double now = clock_->now().seconds();
    const std::string & frame = layered_costmap_->getGlobalFrameID();
    for (const auto & entry : latest) {
      const auto & msg = entry.second;
      if (!isPoseFresh(rclcpp::Time(msg.header.stamp).seconds(), now, max_age)) {
        continue;
      }
      geometry_msgs::msg::PoseStamped in, out;
      in.header.frame_id = msg.header.frame_id;
      in.pose = msg.pose.pose;
      if (in.header.frame_id == frame) {
        out = in;
      } else {
        try {
          // Latest available transform: other robots move slowly compared
          // with how often the costmap updates.
          tf_->transform(in, out, frame, tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 5000, "RobotAvoidanceLayer: cannot transform pose from %s "
            "(%s) to %s: %s", entry.first.c_str(), in.header.frame_id.c_str(), frame.c_str(),
            ex.what());
          continue;
        }
      }
      active_.push_back(out.pose.position);
    }

    const double reach = avoidanceInfluenceRadius(radius, scaling);
    for (const auto & p : active_) {
      Bounds b{p.x - reach, p.y - reach, p.x + reach, p.y + reach};
      expand(b, min_x, min_y, max_x, max_y);
      if (have_last_bounds_) {
        last_bounds_ = {
          std::min(last_bounds_.min_x, b.min_x), std::min(last_bounds_.min_y, b.min_y),
          std::max(last_bounds_.max_x, b.max_x), std::max(last_bounds_.max_y, b.max_y)};
      } else {
        last_bounds_ = b;
        have_last_bounds_ = true;
      }
    }
    active_radius_ = radius;
    active_scaling_ = scaling;
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override
  {
    if (active_.empty()) {
      return;
    }
    const double reach = avoidanceInfluenceRadius(active_radius_, active_scaling_);
    for (const auto & p : active_) {
      int i0, j0, i1, j1;
      master.worldToMapEnforceBounds(p.x - reach, p.y - reach, i0, j0);
      master.worldToMapEnforceBounds(p.x + reach, p.y + reach, i1, j1);
      i0 = std::max(i0, min_i);
      j0 = std::max(j0, min_j);
      i1 = std::min(i1, max_i - 1);   // max_i, max_j are exclusive
      j1 = std::min(j1, max_j - 1);
      for (int j = j0; j <= j1; ++j) {
        for (int i = i0; i <= i1; ++i) {
          double wx, wy;
          master.mapToWorld(i, j, wx, wy);
          const unsigned char cost = computeAvoidanceCost(
            std::hypot(wx - p.x, wy - p.y), active_radius_, active_scaling_);
          if (cost == nav2_costmap_2d::FREE_SPACE) {
            continue;
          }
          const unsigned char old = master.getCost(i, j);
          if (old == nav2_costmap_2d::NO_INFORMATION || cost > old) {
            master.setCost(i, j, cost);
          }
        }
      }
    }
  }

  void reset() override {current_ = true;}

  // Robot poses are not sensor data: clearing the costmap should not drop them.
  bool isClearable() override {return false;}

private:
  struct Bounds
  {
    double min_x, min_y, max_x, max_y;
  };

  static void expand(
    const Bounds & b, double * min_x, double * min_y, double * max_x, double * max_y)
  {
    *min_x = std::min(*min_x, b.min_x);
    *min_y = std::min(*min_y, b.min_y);
    *max_x = std::max(*max_x, b.max_x);
    *max_y = std::max(*max_y, b.max_y);
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & p : parameters) {
      const std::string & n = p.get_name();
      if (n == getFullName("enabled") && p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        enabled_ = p.as_bool();
        RCLCPP_INFO(logger_, "RobotAvoidanceLayer '%s' %s", name_.c_str(),
          enabled_ ? "enabled" : "disabled");
      } else if (n == getFullName("robot_radius") &&  // NOLINT
        p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        robot_radius_ = p.as_double();
      } else if (n == getFullName("cost_scaling") &&  // NOLINT
        p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        cost_scaling_ = p.as_double();
      } else if (n == getFullName("max_pose_age_s") &&  // NOLINT
        p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        max_pose_age_s_ = p.as_double();
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  std::mutex mutex_;   // guards latest_ and the parameters below
  std::map<std::string, geometry_msgs::msg::PoseWithCovarianceStamped> latest_;
  double robot_radius_{0.15};
  double cost_scaling_{3.0};
  double max_pose_age_s_{2.0};

  // Used only from the costmap update thread (updateBounds, then updateCosts).
  std::vector<geometry_msgs::msg::Point> active_;
  double active_radius_{0.15};
  double active_scaling_{3.0};
  Bounds last_bounds_{0.0, 0.0, 0.0, 0.0};
  bool have_last_bounds_{false};

  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
  subscriptions_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace vision_bot_plugins

PLUGINLIB_EXPORT_CLASS(vision_bot_plugins::RobotAvoidanceLayer, nav2_costmap_2d::Layer)
