import { NextResponse } from "next/server";
import { loadRobots } from "../../../lib/fleetQueries";
import { errorResponse, fleet } from "../../../lib/fleetServer";
import { pickRobot } from "../../../lib/robots";

export const dynamic = "force-dynamic";

/**
 * Queue a map-frame goal in the fleet: body {robot: <fleet id> | "nearest-idle", x, y, yaw?}.
 * The robot's fleet_bridge picks it up; the browser polls its status.
 */
export async function POST(request: Request) {
  const body = await request.json().catch(() => null);
  const { robot, x, y, yaw = 0 } = body ?? {};
  const choice = robot === "nearest-idle" ? robot : Number(robot);
  if (![x, y, yaw].every(Number.isFinite) || (choice !== "nearest-idle" && !Number.isInteger(choice))) {
    return NextResponse.json({ error: "expected {robot, x, y, yaw?} with numbers" }, { status: 400 });
  }
  try {
    const target = pickRobot(await loadRobots(fleet(), Date.now()), choice, { x, y });
    if (!target) {
      return NextResponse.json(
        { error: choice === "nearest-idle" ? "No robot is idle." : "Unknown robot." },
        { status: 409 }
      );
    }
    const goal = await fleet().request("POST", `/robots/${target.id}/goals`, { json: { x, y, yaw } });
    return NextResponse.json({ goal, robot: target }, { status: 201 });
  } catch (err) {
    return errorResponse(err);
  }
}
