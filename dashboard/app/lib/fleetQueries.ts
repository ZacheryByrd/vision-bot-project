import type { FleetClient } from "./fleet";
import { summarizeRobot, type RobotView } from "./robots";

const PAGE = 100;

/** Every fleet robot that reports a map-frame pose, with its latest telemetry. */
export async function loadRobots(client: FleetClient, nowMs: number): Promise<RobotView[]> {
  const robots: { id: number; name: string; serial_number: string }[] = [];
  for (let skip = 0; ; skip += PAGE) {
    const page = await client.request("GET", "/robots", { params: { skip, limit: PAGE } });
    robots.push(...page.items);
    if (page.items.length === 0 || robots.length >= page.total) break;
  }
  const views = await Promise.all(
    robots.map(async (robot) => {
      const [latest] = await client.request("GET", `/telemetry/robots/${robot.id}`, {
        params: { limit: 1 },
      });
      return summarizeRobot(robot, latest ?? null, nowMs);
    })
  );
  return views.filter((r) => r.pose !== null);
}

/**
 * One goal of a robot, or null. The fleet API lists a robot's goals oldest
 * first but has no endpoint for a single goal, so this pages through them.
 */
export async function findGoal(client: FleetClient, robotId: number, goalId: number) {
  for (let skip = 0; ; skip += PAGE) {
    const goals = await client.request("GET", `/robots/${robotId}/goals`, {
      params: { skip, limit: PAGE },
    });
    const goal = goals.find((g: { id: number }) => g.id === goalId);
    if (goal) return goal;
    if (goals.length < PAGE) return null;
  }
}
