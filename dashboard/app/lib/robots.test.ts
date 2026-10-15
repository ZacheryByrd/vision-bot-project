import { describe, expect, it } from "vitest";
import { pickRobot, summarizeRobot, type RobotView } from "./robots";

const NOW = Date.parse("2026-10-14T12:00:10Z");

function view(id: number, x: number, y: number, navState: string, ageS = 1): RobotView {
  return {
    id, name: `robot${id}`, serial: `VB-SIM-${id}`,
    pose: { x, y, yaw: 0 }, navState, battery: 90, batterySimulated: true,
    ageS, stale: ageS > 5,
  };
}

describe("summarizeRobot", () => {
  const robot = { id: 1, name: "robot1", serial_number: "VB-SIM-1" };
  const reading = {
    pose_x: 1.5, pose_y: -2, pose_yaw: 0.5, nav_state: "idle",
    battery_level: 97.5, battery_simulated: true, recorded_at: "2026-10-14T12:00:08Z",
  };

  it("combines the robot with its latest reading", () => {
    expect(summarizeRobot(robot, reading, NOW)).toEqual({
      id: 1, name: "robot1", serial: "VB-SIM-1", pose: { x: 1.5, y: -2, yaw: 0.5 },
      navState: "idle", battery: 97.5, batterySimulated: true, ageS: 2, stale: false,
    });
  });

  it("marks a robot stale after 5 s without telemetry", () => {
    const old = { ...reading, recorded_at: "2026-10-14T12:00:04Z" };
    expect(summarizeRobot(robot, old, NOW).stale).toBe(true);
  });

  it("has no pose without a map-frame reading", () => {
    const s = summarizeRobot(robot, { ...reading, pose_x: null, pose_y: null }, NOW);
    expect(s.pose).toBeNull();
    expect(summarizeRobot(robot, null, NOW).pose).toBeNull();
  });
});

describe("pickRobot", () => {
  const robots = [view(1, 0, 0, "idle"), view(2, -2.5, 0, "idle"), view(3, 3, 3, "navigating")];

  it("returns the chosen robot", () => {
    expect(pickRobot(robots, 2, { x: 4, y: 4 })?.id).toBe(2);
  });

  it("picks the nearest idle robot, skipping busy ones", () => {
    expect(pickRobot(robots, "nearest-idle", { x: 3, y: 3 })?.id).toBe(1);
    expect(pickRobot(robots, "nearest-idle", { x: -3, y: 0 })?.id).toBe(2);
  });

  it("skips robots whose telemetry is stale", () => {
    const old = [view(1, 0, 0, "idle", 30), view(2, -2.5, 0, "idle")];
    expect(pickRobot(old, "nearest-idle", { x: 0, y: 0 })?.id).toBe(2);
  });

  it("returns null when no robot is idle or the chosen one is unknown", () => {
    expect(pickRobot([view(3, 3, 3, "navigating")], "nearest-idle", { x: 0, y: 0 })).toBeNull();
    expect(pickRobot(robots, 9, { x: 0, y: 0 })).toBeNull();
  });
});
