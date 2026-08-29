"use client";

import type { CmdVelState, DetectionState } from "../hooks/useRosConnection";

export default function StatusPanel({
  detection,
  cmdVel,
}: {
  detection: DetectionState | null;
  cmdVel: CmdVelState | null;
}) {
  return (
    <div className="status-panel">
      <div className="status-panel__row">
        <span className="status-panel__label">Target detected</span>
        <span className="status-panel__value">
          <span
            className={`status-dot ${
              detection?.detected ? "status-dot--on" : "status-dot--off"
            }`}
          />
          {detection ? (detection.detected ? "yes" : "no") : "--"}
        </span>
      </div>
      <Row
        label="Offset (x, y)"
        value={
          detection
            ? `${detection.offsetX.toFixed(2)}, ${detection.offsetY.toFixed(2)}`
            : "--"
        }
      />
      <Row
        label="Target area"
        value={detection ? `${(detection.areaFraction * 100).toFixed(1)}%` : "--"}
      />
      <Row
        label="cmd_vel (linear, angular)"
        value={
          cmdVel
            ? `${cmdVel.linearX.toFixed(2)} m/s, ${cmdVel.angularZ.toFixed(2)} rad/s`
            : "--"
        }
      />
    </div>
  );
}

function Row({ label, value }: { label: string; value: string }) {
  return (
    <div className="status-panel__row">
      <span className="status-panel__label">{label}</span>
      <span className="status-panel__value">{value}</span>
    </div>
  );
}
