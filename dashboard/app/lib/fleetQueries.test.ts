import { describe, expect, it } from "vitest";
import type { FleetClient } from "./fleet";
import { findGoal, loadRobots } from "./fleetQueries";

/** A fleet client stand-in answering GETs from a table keyed by path + query. */
function fakeClient(table: Record<string, unknown>) {
  const asked: string[] = [];
  const client: FleetClient = {
    request: async (method, path, options) => {
      const query = options?.params ? "?" + new URLSearchParams(
        Object.entries(options.params).map(([k, v]) => [k, String(v)])) : "";
      asked.push(`${method} ${path}${query}`);
      if (!(path + query in table)) throw new Error(`unexpected ${path}${query}`);
      return table[path + query] as any;
    },
  };
  return { client, asked };
}

const reading = (x: number | null) => ({
  pose_x: x, pose_y: 0, pose_yaw: 0, nav_state: "idle", battery_level: 90,
  battery_simulated: true, recorded_at: "2026-10-14T12:00:09Z",
});

describe("loadRobots", () => {
  it("returns the robots that report a map pose, with their latest reading", async () => {
    const { client } = fakeClient({
      "/robots?skip=0&limit=100": {
        items: [{ id: 1, name: "robot1", serial_number: "VB-SIM-1" },
                { id: 2, name: "forklift", serial_number: "FL-1" },
                { id: 3, name: "robot2", serial_number: "VB-SIM-2" }],
        total: 3,
      },
      "/telemetry/robots/1?limit=1": [reading(1.5)],
      "/telemetry/robots/2?limit=1": [reading(null)],
      "/telemetry/robots/3?limit=1": [],
    });
    const robots = await loadRobots(client, Date.parse("2026-10-14T12:00:10Z"));
    expect(robots.map((r) => r.id)).toEqual([1]);
    expect(robots[0].pose).toEqual({ x: 1.5, y: 0, yaw: 0 });
  });
});

describe("findGoal", () => {
  it("pages through the robot's goals until it finds the id", async () => {
    const page1 = Array.from({ length: 100 }, (_, i) => ({ id: i + 1, status: "succeeded" }));
    const { client, asked } = fakeClient({
      "/robots/1/goals?skip=0&limit=100": page1,
      "/robots/1/goals?skip=100&limit=100": [{ id: 101, status: "failed" }],
    });
    expect(await findGoal(client, 1, 101)).toEqual({ id: 101, status: "failed" });
    expect(asked).toHaveLength(2);
  });

  it("returns null when the goal is not there", async () => {
    const { client } = fakeClient({ "/robots/1/goals?skip=0&limit=100": [{ id: 1 }] });
    expect(await findGoal(client, 1, 5)).toBeNull();
  });
});
