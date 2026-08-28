"use client";

import type { CmdVelState, DetectionState } from "../hooks/useRosConnection";

export default function StatusPanel({
  connected,
  detection,
  cmdVel,
}: {
  connected: boolean;
  detection: DetectionState | null;
  cmdVel: CmdVelState | null;
}) {
  return (
    <div className="status-panel">
      <Row label="Connection" value={connected ? "connected" : "disconnected"} />
      <Row
        label="Target detected"
        value={detection ? (detection.detected ? "yes" : "no") : "--"}
      />
      <Row
        label="Offset (x, y)"
        value={
          detection
            ? `${detection.offsetX.toFixed(2)}, ${detection.offsetY.toFixed(2)}`
            : "--"
        }
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
