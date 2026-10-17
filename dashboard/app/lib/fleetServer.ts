import { NextResponse } from "next/server";
import { createFleetClient, FleetError, FleetUnavailable, type FleetClient } from "./fleet";

let client: FleetClient | null = null;

/**
 * The dashboard's fleet API client, created once per server process from
 * FLEET_API_BASE_URL, FLEET_USER and FLEET_PASSWORD (server-only variables in
 * .env.local; none of them is NEXT_PUBLIC, so the browser never sees them).
 */
export function fleet(): FleetClient {
  if (!client) {
    const { FLEET_API_BASE_URL, FLEET_USER, FLEET_PASSWORD } = process.env;
    if (!FLEET_USER || !FLEET_PASSWORD) {
      throw new Error("set FLEET_USER and FLEET_PASSWORD in dashboard/.env.local");
    }
    client = createFleetClient({
      baseUrl: FLEET_API_BASE_URL ?? "http://localhost:8000/api/v1",
      user: FLEET_USER,
      password: FLEET_PASSWORD,
    });
  }
  return client;
}

/** A JSON error for the browser; fleet auth problems are the server's, not the user's. */
export function errorResponse(err: unknown) {
  if (err instanceof FleetUnavailable) {
    console.error(err.message);
    return NextResponse.json({ error: "The fleet API is not answering." }, { status: 503 });
  }
  if (err instanceof FleetError) {
    const status = err.status === 401 || err.status === 403 ? 502 : err.status;
    return NextResponse.json({ error: err.message }, { status });
  }
  return NextResponse.json({ error: String(err) }, { status: 500 });
}
