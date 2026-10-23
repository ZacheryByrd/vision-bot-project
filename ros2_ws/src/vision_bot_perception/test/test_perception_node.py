"""The C++ perception_node as a ROS node: topics, encodings, parameters, failure modes.

Starts three instances: the node under test on /test/image, one with
publish_debug_image:=false on its own topics, and one with the DNN backend
and a model path that does not exist (must exit non-zero at startup).
Expected values come from test/fixtures/reference.json (the Python
detectors' outputs on the same frames).
"""

import json
import time
import unittest
from pathlib import Path

import cv2
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

FIXTURES = Path(__file__).resolve().parent / "fixtures"
CENTERED = next(c for c in json.loads((FIXTURES / "reference.json").read_text())["hsv"]
                if c["image"] == "hsv_centered.png")

node_under_test = launch_ros.actions.Node(
    package="vision_bot_perception", executable="perception_node", name="perception_node",
    parameters=[{"image_topic": "/test/image"}], output="screen")
no_debug = launch_ros.actions.Node(
    package="vision_bot_perception", executable="perception_node", name="perception_nodebug",
    parameters=[{"image_topic": "/nodebug/image", "publish_debug_image": False}],
    remappings=[("/vision_bot/detection", "/nodebug/detection"),
                ("/vision_bot/debug_image", "/nodebug/debug_image")],
    output="screen")
bad_dnn = launch_ros.actions.Node(
    package="vision_bot_perception", executable="perception_node", name="perception_bad_dnn",
    parameters=[{"image_topic": "/bad/image", "detector_backend": "dnn",
                 "dnn_prototxt_path": "/nonexistent/deploy.prototxt",
                 "dnn_model_path": "/nonexistent/model.caffemodel"}],
    output="screen")


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        node_under_test, no_debug, bad_dnn, launch_testing.actions.ReadyToTest()])


class TestPerceptionNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("perception_node_tester")
        cls.bridge = CvBridge()
        cls.frame = cv2.imread(str(FIXTURES / "hsv_centered.png"))
        cls.received = {}
        for topic, kind in [("/vision_bot/detection", Float32MultiArray),
                            ("/vision_bot/debug_image", Image),
                            ("/nodebug/detection", Float32MultiArray),
                            ("/nodebug/debug_image", Image)]:
            cls.received[topic] = []
            cls.node.create_subscription(
                kind, topic, lambda msg, t=topic: cls.received[t].append(msg), 10)
        cls.pubs = {t: cls.node.create_publisher(Image, t, 10)
                    for t in ("/test/image", "/nodebug/image")}

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def image(self, encoding="bgr8"):
        frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB) if encoding == "rgb8" else self.frame
        msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)
        msg.header.frame_id = "camera"
        msg.header.stamp.sec = 1234
        msg.header.stamp.nanosec = 5678
        return msg

    def send_until(self, image_topic, msg, wait_topic, timeout=15.0):
        """Clear wait_topic's messages, publish msg every 0.2 s until one arrives."""
        self.received[wait_topic].clear()
        deadline = time.time() + timeout
        next_send = 0.0
        while time.time() < deadline and not self.received[wait_topic]:
            if time.time() >= next_send:
                self.pubs[image_topic].publish(msg)
                next_send = time.time() + 0.2
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(self.received[wait_topic], f"nothing on {wait_topic} within {timeout} s")
        return self.received[wait_topic][0]

    def assert_matches_reference(self, detection):
        self.assertEqual(len(detection.data), 5)
        for got, want in zip(detection.data, CENTERED["expected"]):
            self.assertAlmostEqual(got, want, places=6)

    def test_centered_red_box_bgr8(self):
        self.assert_matches_reference(
            self.send_until("/test/image", self.image(), "/vision_bot/detection"))
        debug = self.send_until("/test/image", self.image(), "/vision_bot/debug_image")
        self.assertEqual(debug.encoding, "bgr8")
        self.assertEqual((debug.header.stamp.sec, debug.header.stamp.nanosec), (1234, 5678))
        self.assertEqual(debug.header.frame_id, "camera")
        self.assertEqual((debug.width, debug.height), (self.frame.shape[1], self.frame.shape[0]))

    def test_rgb8_input_is_converted(self):
        self.assert_matches_reference(
            self.send_until("/test/image", self.image("rgb8"), "/vision_bot/detection"))

    def set_min_area(self, value):
        client = self.node.create_client(SetParameters, "/perception_node/set_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        param = Parameter(name="min_contour_area", value=ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=value))
        future = client.call_async(SetParameters.Request(parameters=[param]))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().results[0].successful)
        self.node.destroy_client(client)

    def test_param_set_takes_effect(self):
        self.set_min_area(100000)
        try:
            detection = self.send_until("/test/image", self.image(), "/vision_bot/detection")
            self.assertEqual(detection.data[0], 0.0)
        finally:
            self.set_min_area(200)

    def test_no_debug_image_when_disabled(self):
        self.send_until("/nodebug/image", self.image(), "/nodebug/detection")
        self.received["/nodebug/debug_image"].clear()
        end = time.time() + 2.0
        while time.time() < end:
            self.pubs["/nodebug/image"].publish(self.image())
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertEqual(self.received["/nodebug/debug_image"], [])

    def test_dnn_bad_model_path_fails_at_startup(self, proc_info, proc_output):
        proc_info.assertWaitForShutdown(process=bad_dnn, timeout=10)
        # A clean error exit, not an abort: an abort leaves a crash dump (and on
        # WSL hands it to the crash-capture helper, which can take longer than this).
        self.assertEqual(proc_info[bad_dnn].returncode, 1)
        proc_output.assertWaitFor("/nonexistent/model.caffemodel", process=bad_dnn, timeout=5)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_good_nodes_exit_cleanly(self, proc_info):
        for process in (node_under_test, no_debug):
            launch_testing.asserts.assertExitCodes(
                proc_info, process=process,
                allowable_exit_codes=[0, -2, -15, launch_testing.asserts.EXIT_OK])
