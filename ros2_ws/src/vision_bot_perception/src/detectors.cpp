#include "vision_bot_perception/detectors.hpp"

#include <algorithm>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace vision_bot_perception
{

namespace
{

// Offsets and area from a target box, plus the debug drawing; shared by both
// detectors, as in the Python node.
Detection from_box(
  const cv::Mat & frame, double x, double y, double bw, double bh, double area,
  double confidence, cv::Mat * debug)
{
  const double w = frame.cols;
  const double h = frame.rows;
  const double cx = x + bw / 2.0;
  const double cy = y + bh / 2.0;

  Detection d;
  d.detected = true;
  d.offset_x = (cx - w / 2.0) / (w / 2.0);
  d.offset_y = (cy - h / 2.0) / (h / 2.0);
  d.area_fraction = std::min(1.0, area / (w * h));
  d.confidence = confidence;

  if (debug != nullptr) {
    // Python's int() truncates; static_cast<int> does the same for these
    // non-negative pixel coordinates.
    cv::rectangle(*debug, cv::Point(static_cast<int>(x), static_cast<int>(y)),
      cv::Point(static_cast<int>(x + bw), static_cast<int>(y + bh)), cv::Scalar(0, 255, 0), 2);
    cv::circle(*debug, cv::Point(static_cast<int>(cx), static_cast<int>(cy)), 5,
      cv::Scalar(0, 0, 255), -1);
    cv::line(*debug, cv::Point(frame.cols / 2, 0), cv::Point(frame.cols / 2, frame.rows),
      cv::Scalar(255, 0, 0), 1);
  }
  return d;
}

}  // namespace

Detection detect_target(const cv::Mat & bgr, const HsvParams & params, cv::Mat * debug)
{
  if (debug != nullptr) {
    *debug = bgr.clone();
  }

  cv::Mat hsv, mask;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(params.hue_low, params.sat_low, params.val_low),
    cv::Scalar(params.hue_high, 255, 255), mask);
  // Python's erode/dilate(mask, None, iterations=2): default 3x3 kernel.
  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {
    return Detection{};
  }

  // Python's max() keeps the first of equal areas; so does max_element.
  const auto largest = std::max_element(contours.begin(), contours.end(),
      [](const auto & a, const auto & b) {return cv::contourArea(a) < cv::contourArea(b);});
  const double area = cv::contourArea(*largest);
  if (area < params.min_area) {
    return Detection{};
  }

  const cv::Rect box = cv::boundingRect(*largest);
  return from_box(bgr, box.x, box.y, box.width, box.height, area, 1.0, debug);
}

Detection dnn_detect_target(
  const cv::Mat & bgr, cv::dnn::Net & net, int target_class_id,
  double confidence_threshold, cv::Mat * debug)
{
  if (debug != nullptr) {
    *debug = bgr.clone();
  }

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(300, 300));
  // Mean Scalar(127.5) subtracts 127.5 from the first (blue) channel only.
  // That is what the Python node's bare `127.5` does too (OpenCV's Python
  // binding turns a number into Scalar(127.5, 0, 0, 0)), and matching it is
  // the point here; Scalar(127.5, 127.5, 127.5) shifts the person box by up
  // to 0.017 in offset and fails test_detectors' reference check.
  const cv::Mat blob = cv::dnn::blobFromImage(
    resized, 0.007843, cv::Size(300, 300), cv::Scalar(127.5), false, false);
  net.setInput(blob);
  // Output shape [1, 1, N, 7]: per detection
  // (image_id, class_id, confidence, x1, y1, x2, y2), box in [0, 1].
  const cv::Mat out = net.forward();
  const cv::Mat rows(out.size[2], out.size[3], CV_32F, const_cast<float *>(out.ptr<float>()));

  double best_confidence = 0.0;
  int best = -1;
  for (int i = 0; i < rows.rows; ++i) {
    const double confidence = rows.at<float>(i, 2);
    const int class_id = static_cast<int>(rows.at<float>(i, 1));
    // Strictly greater, like the Python loop: the first of equal scores wins.
    if (class_id == target_class_id && confidence >= confidence_threshold &&
      confidence > best_confidence)
    {
      best_confidence = confidence;
      best = i;
    }
  }
  if (best < 0) {
    return Detection{};
  }

  const double w = bgr.cols;
  const double h = bgr.rows;
  const double x1 = rows.at<float>(best, 3) * w;
  const double y1 = rows.at<float>(best, 4) * h;
  const double x2 = rows.at<float>(best, 5) * w;
  const double y2 = rows.at<float>(best, 6) * h;
  const double bw = x2 - x1;
  const double bh = y2 - y1;
  return from_box(bgr, x1, y1, bw, bh, bw * bh, best_confidence, debug);
}

}  // namespace vision_bot_perception
