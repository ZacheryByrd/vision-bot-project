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
const RECONNECT_BASE_DELAY_MS = 2000;
const RECONNECT_MAX_DELAY_MS = 10000;

export function useRosConnection() {
  const [connected, setConnected] = useState(false);
  const [detection, setDetection] = useState<DetectionState | null>(null);
  const [cmdVel, setCmdVel] = useState<CmdVelState | null>(null);
  // motor_control_node has no "current mode" topic, only the command
  // topic below -- this tracks the last value we sent, not a confirmed
  // ack from the robot. Defaults to true to match the node's own default.
  const [autonomousEnabled, setAutonomousEnabledState] = useState(true);
  const rosRef = useRef<ROSLIB.Ros | null>(null);
  const autonomousTopicRef = useRef<ROSLIB.Topic | null>(null);

  useEffect(() => {
    let cancelled = false;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    let reconnectAttempt = 0;

    function connect() {
      if (cancelled) {
        return;
      }

      const ros = new ROSLIB.Ros({ url: ROS_WS_URL });
      rosRef.current = ros;

      let reconnectScheduled = false;
      function scheduleReconnect() {
        setConnected(false);
        autonomousTopicRef.current = null;
        if (cancelled || reconnectScheduled) {
          return;
        }
        // The underlying WebSocket can fire both "error" and "close" for
        // the same drop -- only schedule one retry per connection attempt.
        reconnectScheduled = true;
        const delay = Math.min(
          RECONNECT_MAX_DELAY_MS,
          RECONNECT_BASE_DELAY_MS * 2 ** reconnectAttempt
        );
        reconnectAttempt += 1;
        reconnectTimer = setTimeout(connect, delay);
      }

      ros.on("connection", () => {
        reconnectAttempt = 0;
        setConnected(true);
      });
      ros.on("close", scheduleReconnect);
      ros.on("error", scheduleReconnect);

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

      autonomousTopicRef.current = new ROSLIB.Topic({
        ros,
        name: "/vision_bot/autonomous_enabled",
        messageType: "std_msgs/Bool",
      });
    }

    connect();

    return () => {
      cancelled = true;
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
      }
      autonomousTopicRef.current = null;
      rosRef.current?.close();
    };
  }, []);

  function setAutonomousEnabled(enabled: boolean) {
    autonomousTopicRef.current?.publish(new ROSLIB.Message({ data: enabled }));
    setAutonomousEnabledState(enabled);
  }

  return { connected, detection, cmdVel, autonomousEnabled, setAutonomousEnabled };
}
