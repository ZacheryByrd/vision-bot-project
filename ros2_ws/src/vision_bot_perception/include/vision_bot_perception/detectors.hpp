// The two target detectors behind perception_node, ported from
// vision_bot/perception_node.py (detect_target and dnn_detect_target).
// No ROS here: test/test_detectors.cpp checks them against the Python
// functions' outputs on the same frames.
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

namespace vision_bot_perception
{

// What both detectors return; the node publishes it as
// [detected, offset_x, offset_y, area_fraction, confidence].
// Offsets are the target center's offset from the image center, normalized
// to [-1, 1] (negative = left / above).
struct Detection
{
  bool detected = false;
  double offset_x = 0.0;
  double offset_y = 0.0;
  double area_fraction = 0.0;
  double confidence = 0.0;
};

struct HsvParams
{
  int hue_low = 0;
  int hue_high = 10;
  int sat_low = 120;
  int val_low = 80;
  double min_area = 200.0;
};

// HSV threshold + largest contour. Confidence is 1.0 when detected (the HSV
// detector has no real score). If debug is not null it receives a copy of
// the frame with the box, centroid and center line drawn on it.
Detection detect_target(const cv::Mat & bgr, const HsvParams & params, cv::Mat * debug);

// MobileNet-SSD (Caffe) via cv::dnn: the highest-scoring detection of
// target_class_id at or above confidence_threshold. Confidence is the model
// score. net is loaded once by the caller (readNetFromCaffe). Same debug
// drawing as detect_target.
Detection dnn_detect_target(
  const cv::Mat & bgr, cv::dnn::Net & net, int target_class_id,
  double confidence_threshold, cv::Mat * debug);

}  // namespace vision_bot_perception
