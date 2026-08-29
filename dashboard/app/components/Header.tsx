"use client";

export default function Header({ connected }: { connected: boolean }) {
  return (
    <header className="header">
      <div className="header__title">
        <span className="header__name">vision_bot</span>
        <span className="header__subtitle">live dashboard</span>
      </div>
      <div className={`connection-pill ${connected ? "connection-pill--up" : "connection-pill--down"}`}>
        <span className="connection-pill__dot" />
        {connected ? "connected" : "disconnected"}
      </div>
    </header>
  );
}
