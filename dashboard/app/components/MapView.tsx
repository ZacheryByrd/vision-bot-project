"use client";

import { useEffect, useRef, useState, type MouseEvent } from "react";
import metaJson from "../../public/maps/nav_world.json";
import { isInsideMap, pixelToWorld, worldToPixel, type MapMeta } from "../lib/mapTransform";
import type { RobotView } from "../lib/robots";

const META = metaJson as MapMeta;
const ROBOT_RADIUS_PX = 0.145 / META.resolution; // the robot's circumscribed radius
const FINISHED = ["succeeded", "failed", "canceled"];

type GoalEntry = {
  id: number;
  robotId: number;
  robotName: string;
  x: number;
  y: number;
  status: string;
};

/**
 * The saved map with each robot's pose from fleet telemetry; a click queues a
 * goal in the fleet for the chosen robot (or the nearest idle one) through the
 * dashboard's server routes, never through rosbridge.
 */
export default function MapView() {
  const [robots, setRobots] = useState<RobotView[]>([]);
  const [fleetError, setFleetError] = useState<string | null>(null);
  const [choice, setChoice] = useState<string>("nearest-idle");
  const [goals, setGoals] = useState<GoalEntry[]>([]);
  const [notice, setNotice] = useState<string | null>(null);
  const svgRef = useRef<SVGSVGElement>(null);

  useEffect(() => {
    let cancelled = false;
    async function poll() {
      try {
        const resp = await fetch("/api/fleet/robots", { cache: "no-store" });
        const body = await resp.json();
        if (cancelled) return;
        if (!resp.ok) throw new Error(body.error ?? resp.statusText);
        setRobots(body);
        setFleetError(null);
      } catch (err) {
        if (!cancelled) setFleetError(String(err instanceof Error ? err.message : err));
      }
    }
    poll();
    const timer = setInterval(poll, 1000);
    return () => {
      cancelled = true;
      clearInterval(timer);
    };
  }, []);

  // Follow every unfinished goal until the fleet reports how it ended.
  useEffect(() => {
    const open = goals.filter((g) => !FINISHED.includes(g.status));
    if (open.length === 0) return;
    const timer = setInterval(async () => {
      for (const g of open) {
        const resp = await fetch(`/api/fleet/goals/${g.robotId}/${g.id}`, { cache: "no-store" });
        if (!resp.ok) continue;
        const { status } = await resp.json();
        setGoals((all) => all.map((e) => (e.id === g.id ? { ...e, status } : e)));
      }
    }, 1000);
    return () => clearInterval(timer);
  }, [goals]);

  async function onMapClick(event: MouseEvent<SVGSVGElement>) {
    const svg = svgRef.current;
    const ctm = svg?.getScreenCTM();
    if (!svg || !ctm) return;
    const point = new DOMPoint(event.clientX, event.clientY).matrixTransform(ctm.inverse());
    if (!isInsideMap(point.x, point.y, META)) return;
    const { x, y } = pixelToWorld(point.x, point.y, META);
    setNotice(null);
    const resp = await fetch("/api/fleet/goals", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ robot: choice === "nearest-idle" ? choice : Number(choice), x, y }),
    });
    const body = await resp.json();
    if (!resp.ok) {
      setNotice(body.error ?? "The goal was not queued.");
      return;
    }
    setGoals((all) =>
      [{ id: body.goal.id, robotId: body.robot.id, robotName: body.robot.name, x, y,
         status: body.goal.status }, ...all].slice(0, 6)
    );
  }

  const active = goals.filter((g) => !FINISHED.includes(g.status));

  return (
    <section className="map-view">
      <div className="map-view__header">
        <span className="map-view__title">Map</span>
        <label className="map-view__choice">
          Send to{" "}
          <select value={choice} onChange={(e) => setChoice(e.target.value)}>
            <option value="nearest-idle">nearest idle robot</option>
            {robots.map((r) => (
              <option key={r.id} value={r.id}>{r.name}</option>
            ))}
          </select>
        </label>
      </div>

      <div className="map-view__canvas">
        {/* eslint-disable-next-line @next/next/no-img-element */}
        <img src="/maps/nav_world.png" alt="Saved map of the navigation world" />
        <svg
          ref={svgRef}
          viewBox={`0 0 ${META.widthPx} ${META.heightPx}`}
          onClick={onMapClick}
          aria-label="Click to send a goal"
        >
          {active.map((g) => {
            const p = worldToPixel(g.x, g.y, META);
            return (
              <g key={g.id} className="map-view__goal">
                <line x1={p.px - 3} y1={p.py - 3} x2={p.px + 3} y2={p.py + 3} />
                <line x1={p.px - 3} y1={p.py + 3} x2={p.px + 3} y2={p.py - 3} />
              </g>
            );
          })}
          {robots.map((r) => {
            if (!r.pose) return null;
            const p = worldToPixel(r.pose.x, r.pose.y, META);
            const tip = {
              px: p.px + Math.cos(r.pose.yaw) * ROBOT_RADIUS_PX * 1.8,
              py: p.py - Math.sin(r.pose.yaw) * ROBOT_RADIUS_PX * 1.8,
            };
            const state = r.stale ? "stale" : r.navState === "navigating" ? "navigating" : "idle";
            return (
              <g key={r.id} className={`map-view__robot map-view__robot--${state}`}>
                <circle cx={p.px} cy={p.py} r={ROBOT_RADIUS_PX} />
                <line x1={p.px} y1={p.py} x2={tip.px} y2={tip.py} />
                <text x={p.px + ROBOT_RADIUS_PX + 1.5} y={p.py - ROBOT_RADIUS_PX}>{r.name}</text>
              </g>
            );
          })}
        </svg>
      </div>

      {fleetError && <p className="map-view__error">Fleet: {fleetError}</p>}
      {notice && <p className="map-view__error">{notice}</p>}

      <div className="map-view__robots">
        {robots.map((r) => (
          <div key={r.id} className="status-panel__row">
            <span className="status-panel__label">{r.name}</span>
            <span className="status-panel__value">
              {r.stale ? "no recent telemetry" : r.navState}
              {r.battery !== null && ` · ${r.battery.toFixed(0)}%${r.batterySimulated ? " (sim)" : ""}`}
            </span>
          </div>
        ))}
      </div>

      {goals.length > 0 && (
        <ul className="map-view__goals">
          {goals.map((g) => (
            <li key={g.id}>
              <span>
                Goal {g.id} → {g.robotName} ({g.x.toFixed(2)}, {g.y.toFixed(2)})
              </span>
              <span className={`goal-status goal-status--${g.status}`}>{g.status}</span>
            </li>
          ))}
        </ul>
      )}
      <p className="control-panel__hint">
        Click the map to send a goal. Goals go through the fleet platform; each
        robot&apos;s fleet bridge picks them up and reports progress.
      </p>
    </section>
  );
}
