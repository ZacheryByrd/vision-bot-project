// The C++ detectors against the Python ones: test/fixtures holds frames and
// what vision_bot/perception_node.py returned for each
// (scripts/make_perception_fixtures.py).
#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vision_bot_perception/detectors.hpp"

using vision_bot_perception::Detection;
using vision_bot_perception::HsvParams;
using vision_bot_perception::detect_target;
using vision_bot_perception::dnn_detect_target;

namespace
{

nlohmann::json reference()
{
  std::ifstream in(std::string(FIXTURE_DIR) + "/reference.json");
  return nlohmann::json::parse(in);
}

// "vision_bot/..." paths are in the vision_bot package (the person photo);
// the rest are in test/fixtures.
std::string fixture_path(const std::string & name)
{
  const std::string prefix = "vision_bot/";
  if (name.rfind(prefix, 0) == 0) {
    return std::string(VISION_BOT_SRC_DIR) + "/" + name.substr(prefix.size());
  }
  return std::string(FIXTURE_DIR) + "/" + name;
}

cv::Mat fixture(const std::string & name)
{
  cv::Mat frame = cv::imread(fixture_path(name), cv::IMREAD_COLOR);
  EXPECT_FALSE(frame.empty()) << name;
  return frame;
}

void expect_matches(const Detection & d, const nlohmann::json & expected, double tol,
  const std::string & name)
{
  EXPECT_EQ(d.detected ? 1.0 : 0.0, expected[0].get<double>()) << name;
  EXPECT_NEAR(d.offset_x, expected[1].get<double>(), tol) << name;
  EXPECT_NEAR(d.offset_y, expected[2].get<double>(), tol) << name;
  EXPECT_NEAR(d.area_fraction, expected[3].get<double>(), tol) << name;
  EXPECT_NEAR(d.confidence, expected[4].get<double>(), tol) << name;
}

}  // namespace

TEST(HsvReference, MatchesPythonOnEveryFixture)
{
  const auto cases = reference()["hsv"];
  ASSERT_GE(cases.size(), 11u);
  for (const auto & c : cases) {
    const auto & p = c["params"];
    HsvParams params;
    params.hue_low = p["hue_low"];
    params.hue_high = p["hue_high"];
    params.sat_low = p["sat_low"];
    params.val_low = p["val_low"];
    params.min_area = p["min_area"];
    const std::string name = c["image"];
    expect_matches(detect_target(fixture(name), params, nullptr), c["expected"], 1e-6, name);
  }
}

TEST(HsvDetector, DebugFrameSameSizeAsInput)
{
  const cv::Mat frame = fixture("hsv_centered.png");
  cv::Mat debug;
  detect_target(frame, HsvParams{}, &debug);
  EXPECT_EQ(debug.size(), frame.size());
  EXPECT_EQ(debug.type(), frame.type());
}

TEST(HsvDetector, NullDebugDoesNotCrash)
{
  EXPECT_TRUE(detect_target(fixture("hsv_centered.png"), HsvParams{}, nullptr).detected);
}

TEST(DnnReference, MatchesPythonOnEveryFixture)
{
  const std::string models = std::string(VISION_BOT_SRC_DIR) + "/models/dnn/";
  const std::string prototxt = models + "MobileNetSSD_deploy.prototxt";
  const std::string model = models + "MobileNetSSD_deploy.caffemodel";
  if (!std::ifstream(prototxt) || !std::ifstream(model)) {
    GTEST_SKIP() << "MobileNetSSD model files not present in " << models;
  }
  cv::dnn::Net net = cv::dnn::readNetFromCaffe(prototxt, model);

  const auto cases = reference()["dnn"];
  ASSERT_GE(cases.size(), 4u);
  for (const auto & c : cases) {
    const std::string name = c["image"];
    const Detection d = dnn_detect_target(fixture(name), net, c["target_class_id"].get<int>(),
        c["confidence_threshold"].get<double>(), nullptr);
    expect_matches(d, c["expected"], 1e-3,
      name + " class " + c["target_class_id"].dump() + " threshold " +
      c["confidence_threshold"].dump());
  }
}
