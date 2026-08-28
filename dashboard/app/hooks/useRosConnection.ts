"use client";

import { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";

export type DetectionState = {
  detected: boolean;
  offsetX: number;
  offsetY: number;
  areaFraction: number;
  confidence: number;
};

export type CmdVelState = {
  linearX: number;
  angularZ: number;
};

const ROS_WS_URL =
  process.env.NEXT_PUBLIC_ROS_WS_URL || "ws://localhost:9090";

/**
 * Opens a rosbridge WebSocket connection and subscribes to the topics
 * the dashboard cares about. Mirrors the field layout published by
 * perception_node.py / motor_control_node.py in the ROS2 package --
 * keep these in sync if you change the message layout there.
 */
export function useRosConnection() {
  const [connected, setConnected] = useState(false);
  const [detection, setDetection] = useState<DetectionState | null>(null);
  const [cmdVel, setCmdVel] = useState<CmdVelState | null>(null);
  const rosRef = useRef<ROSLIB.Ros | null>(null);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: ROS_WS_URL });
    rosRef.current = ros;

    ros.on("connection", () => setConnected(true));
    ros.on("close", () => setConnected(false));
    ros.on("error", () => setConnected(false));

    const detectionTopic = new ROSLIB.Topic({
      ros,
      name: "/vision_bot/detection",
      messageType: "std_msgs/Float32MultiArray",
    });
    detectionTopic.subscribe((msg: any) => {
      const [detected, offsetX, offsetY, areaFraction, confidence] = msg.data;
      setDetection({
        detected: detected > 0.5,
        offsetX,
        offsetY,
        areaFraction,
        confidence,
      });
    });

    const cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: "/cmd_vel",
      messageType: "geometry_msgs/Twist",
    });
    cmdVelTopic.subscribe((msg: any) => {
      setCmdVel({ linearX: msg.linear.x, angularZ: msg.angular.z });
    });

    return () => {
      detectionTopic.unsubscribe();
      cmdVelTopic.unsubscribe();
      ros.close();
    };
  }, []);

  return { connected, detection, cmdVel };
}
