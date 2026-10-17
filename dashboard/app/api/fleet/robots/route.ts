import { NextResponse } from "next/server";
import { loadRobots } from "../../../lib/fleetQueries";
import { errorResponse, fleet } from "../../../lib/fleetServer";

export const dynamic = "force-dynamic";

/** Robots that report a map pose, with their latest telemetry. */
export async function GET() {
  try {
    return NextResponse.json(await loadRobots(fleet(), Date.now()));
  } catch (err) {
    return errorResponse(err);
  }
}
