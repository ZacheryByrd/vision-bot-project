"use client";

import { useRosConnection } from "./hooks/useRosConnection";
import VideoFeed from "./components/VideoFeed";
import StatusPanel from "./components/StatusPanel";
import ControlPanel from "./components/ControlPanel";

export default function DashboardPage() {
  const { connected, detection, cmdVel, autonomousEnabled, setAutonomousEnabled } =
    useRosConnection();

  return (
    <main className="dashboard">
      <h1>vision_bot dashboard</h1>
      <div className="dashboard__grid">
        <VideoFeed connected={connected} />
        <div className="dashboard__sidebar">
          <StatusPanel connected={connected} detection={detection} cmdVel={cmdVel} />
          <ControlPanel
            connected={connected}
            autonomousEnabled={autonomousEnabled}
            onToggle={setAutonomousEnabled}
          />
        </div>
      </div>
    </main>
  );
}
