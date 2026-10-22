// perception_node in C++: the same node as vision_bot/perception_node.py
// (name, topics, message layout, parameters), with the detectors from
// detectors.cpp. Owns the camera stream; knows nothing about motors.
//
// Subscribes  image_topic (default /camera/image_raw, sensor_msgs/Image, any
//             encoding cv_bridge can convert to bgr8)
// Publishes   /vision_bot/detection   std_msgs/Float32MultiArray
//               [detected, offset_x, offset_y, area_fraction, confidence]
//             /vision_bot/debug_image sensor_msgs/Image (bgr8, input header)
//
// Built as a component (vision_bot_perception::PerceptionNode) and, through
// make_perception_node and perception_main.cpp, as the standalone executable
// perception_node.

#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/dnn.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "vision_bot_perception/detectors.hpp"
#include "vision_bot_perception/perception_node.hpp"

namespace vision_bot_perception
{

class PerceptionNode : public rclcpp::Node
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options)
  : Node("perception_node", options)
  {
    const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    declare_parameter<int>("target_hue_low", 0);  // default: red target
    declare_parameter<int>("target_hue_high", 10);
    declare_parameter<int>("target_sat_low", 120);
    declare_parameter<int>("target_val_low", 80);
    declare_parameter<int>("min_contour_area", 200);  // px^2, filters noise
    declare_parameter<bool>("publish_debug_image", true);

    // "hsv" (default) or "dnn" (MobileNet-SSD, see dnn_follow_launch.py).
    const auto backend = declare_parameter<std::string>("detector_backend", "hsv");
    const auto prototxt = declare_parameter<std::string>("dnn_prototxt_path", "");
    const auto model = declare_parameter<std::string>("dnn_model_path", "");
    declare_parameter<int>("dnn_target_class_id", 15);  // 15 = "person" in the VOC classes
    declare_parameter<double>("dnn_confidence_threshold", 0.5);

    if (backend == "dnn") {
      // Like the Python node, a model that cannot be loaded stops the node
      // at startup instead of running without detections.
      try {
        net_ = cv::dnn::readNetFromCaffe(prototxt, model);
      } catch (const cv::Exception & e) {
        RCLCPP_FATAL(get_logger(), "cannot load DNN model %s / %s: %s",
          prototxt.c_str(), model.c_str(), e.what());
        throw std::runtime_error("cannot load DNN model " + prototxt + " / " + model);
      }
      use_dnn_ = true;
      RCLCPP_INFO(get_logger(), "loaded DNN model: %s / %s", prototxt.c_str(), model.c_str());
    }

    detection_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/vision_bot/detection", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision_bot/debug_image", 10);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, 10,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(msg);});

    RCLCPP_INFO(get_logger(),
      "perception_node up, subscribed to '%s', publishing detections on "
      "'/vision_bot/detection' (C++)", image_topic.c_str());
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    const cv_bridge::CvImageConstPtr frame = cv_bridge::toCvShare(msg, "bgr8");
    // Read on every frame, as the Python node does, so `ros2 param set`
    // takes effect at once.
    const bool publish_debug = get_parameter("publish_debug_image").as_bool();
    cv::Mat debug;
    const Detection d = detect(frame->image, publish_debug ? &debug : nullptr);

    std_msgs::msg::Float32MultiArray out;
    out.data = {
      d.detected ? 1.0f : 0.0f,
      static_cast<float>(d.offset_x),
      static_cast<float>(d.offset_y),
      static_cast<float>(d.area_fraction),
      static_cast<float>(d.confidence)};
    detection_pub_->publish(out);

    if (publish_debug) {
      debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg());
    }
  }

  Detection detect(const cv::Mat & frame, cv::Mat * debug)
  {
    if (use_dnn_) {
      return dnn_detect_target(frame, net_,
        static_cast<int>(get_parameter("dnn_target_class_id").as_int()),
        get_parameter("dnn_confidence_threshold").as_double(), debug);
    }
    HsvParams params;
    params.hue_low = static_cast<int>(get_parameter("target_hue_low").as_int());
    params.hue_high = static_cast<int>(get_parameter("target_hue_high").as_int());
    params.sat_low = static_cast<int>(get_parameter("target_sat_low").as_int());
    params.val_low = static_cast<int>(get_parameter("target_val_low").as_int());
    params.min_area = static_cast<double>(get_parameter("min_contour_area").as_int());
    return detect_target(frame, params, debug);
  }

  bool use_dnn_ = false;
  cv::dnn::Net net_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

rclcpp::Node::SharedPtr make_perception_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<PerceptionNode>(options);
}

}  // namespace vision_bot_perception

RCLCPP_COMPONENTS_REGISTER_NODE(vision_bot_perception::PerceptionNode)
