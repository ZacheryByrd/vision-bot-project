"use client";

import { useRosConnection } from "./hooks/useRosConnection";
import VideoFeed from "./components/VideoFeed";
import StatusPanel from "./components/StatusPanel";

export default function DashboardPage() {
  const { connected, detection, cmdVel } = useRosConnection();

  return (
    <main className="dashboard">
      <h1>vision_bot dashboard</h1>
      <div className="dashboard__grid">
        <VideoFeed connected={connected} />
        <StatusPanel connected={connected} detection={detection} cmdVel={cmdVel} />
      </div>
    </main>
  );
}
