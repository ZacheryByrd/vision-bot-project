/** What the map view shows about each robot, built from the fleet's robot and telemetry. */

export const STALE_AFTER_S = 5;

export type RobotView = {
  id: number;
  name: string;
  serial: string;
  /** Map-frame pose from the latest telemetry, or null if the robot reports none. */
  pose: { x: number; y: number; yaw: number } | null;
  navState: string | null;
  battery: number | null;
  batterySimulated: boolean | null;
  /** Seconds since the latest telemetry, or null if there is none. */
  ageS: number | null;
  stale: boolean;
};

type FleetRobot = { id: number; name: string; serial_number: string };
type FleetReading = {
  pose_x: number | null;
  pose_y: number | null;
  pose_yaw: number | null;
  nav_state: string | null;
  battery_level: number | null;
  battery_simulated: boolean | null;
  recorded_at: string;
};

export function summarizeRobot(robot: FleetRobot, reading: FleetReading | null, nowMs: number): RobotView {
  const ageS = reading ? (nowMs - Date.parse(reading.recorded_at)) / 1000 : null;
  const hasPose = reading !== null && reading.pose_x !== null && reading.pose_y !== null;
  return {
    id: robot.id,
    name: robot.name,
    serial: robot.serial_number,
    pose: hasPose ? { x: reading!.pose_x!, y: reading!.pose_y!, yaw: reading!.pose_yaw ?? 0 } : null,
    navState: reading?.nav_state ?? null,
    battery: reading?.battery_level ?? null,
    batterySimulated: reading?.battery_simulated ?? null,
    ageS,
    stale: ageS === null || ageS > STALE_AFTER_S,
  };
}

/**
 * The robot a clicked goal goes to: the chosen one, or for "nearest-idle" the
 * closest robot that is idle with fresh telemetry. Null if there is none.
 */
export function pickRobot(
  robots: RobotView[],
  choice: number | "nearest-idle",
  target: { x: number; y: number }
): RobotView | null {
  if (choice !== "nearest-idle") return robots.find((r) => r.id === choice) ?? null;
  const idle = robots.filter((r) => r.pose && !r.stale && r.navState === "idle");
  const dist = (r: RobotView) => Math.hypot(r.pose!.x - target.x, r.pose!.y - target.y);
  return idle.sort((a, b) => dist(a) - dist(b))[0] ?? null;
}
