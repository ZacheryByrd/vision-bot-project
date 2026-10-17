import { NextResponse } from "next/server";
import { findGoal } from "../../../../../lib/fleetQueries";
import { errorResponse, fleet } from "../../../../../lib/fleetServer";

export const dynamic = "force-dynamic";

/** The current status of one goal. */
export async function GET(_: Request, { params }: { params: { robotId: string; goalId: string } }) {
  try {
    const goal = await findGoal(fleet(), Number(params.robotId), Number(params.goalId));
    if (!goal) return NextResponse.json({ error: "Goal not found." }, { status: 404 });
    return NextResponse.json(goal);
  } catch (err) {
    return errorResponse(err);
  }
}
