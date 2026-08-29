"use client";

export default function ControlPanel({
  connected,
  autonomousEnabled,
  onToggle,
}: {
  connected: boolean;
  autonomousEnabled: boolean;
  onToggle: (enabled: boolean) => void;
}) {
  return (
    <div className="control-panel">
      <div className="control-panel__label">
        Mode: {autonomousEnabled ? "autonomous" : "manual"}
      </div>
      <button
        className="control-panel__button"
        disabled={!connected}
        onClick={() => onToggle(!autonomousEnabled)}
      >
        {autonomousEnabled ? "Switch to manual" : "Switch to autonomous"}
      </button>
      <p className="control-panel__hint">
        Manual mode frees up /cmd_vel for teleop_twist_keyboard or another
        publisher -- motor_control_node stops publishing entirely while
        disabled, rather than fighting for the topic.
      </p>
    </div>
  );
}
