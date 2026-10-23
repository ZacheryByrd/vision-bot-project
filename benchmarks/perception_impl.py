#!/usr/bin/env python3
"""perception_node in Python vs C++ on the same camera frames.

Two steps, so the simulator is not competing for CPU while the nodes are
measured:

    # 1. with a simulation running (sim_launch.py, or dnn_follow_launch.py
    #    for --backend dnn), record every 3rd camera frame, as received
    #    (Gazebo publishes rgb8, so both nodes also do the bgr8 conversion)
    python3 benchmarks/perception_impl.py capture --out /tmp/frames_hsv.npz
    # 2. with the simulation stopped
    python3 benchmarks/perception_impl.py run --frames /tmp/frames_hsv.npz --backend hsv

`run` starts each implementation in turn as its own process on
/bench/image (detections remapped to /bench/detection), alternating the
order over --rounds, and measures:
    latency  lockstep: publish one frame, wait for its detection (2 s
             timeout); publish to callback, measured in this process
    CPU      the frames streamed at --rate Hz for --stream-s seconds; the
             node process's utime+stime from /proc/<pid>/stat over wall time
    parity   the latency pass's detections compared frame by frame
             between the implementations

Writes benchmarks/results/perception_impl_<backend>_<timestamp>.{csv,md}.
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from provenance import git_commit

RESULTS = Path(__file__).resolve().parent / "results"
IMPLS = {"python": "vision_bot", "cpp": "vision_bot_perception"}
BENCH_IMAGE = "/bench/image"
BENCH_DETECTION = "/bench/detection"


# --- pure helpers (tested in tests/test_perception_impl.py) ---------------

def _percentile(sorted_values, q):
    """Linear interpolation between closest ranks (numpy's default)."""
    pos = (len(sorted_values) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(sorted_values) - 1)
    return sorted_values[lo] + (sorted_values[hi] - sorted_values[lo]) * (pos - lo)


def summarize(latencies_s):
    """n, mean, p50, p95 and max of latencies given in seconds, in milliseconds."""
    if not latencies_s:
        raise ValueError("no latencies")
    ms = sorted(v * 1000.0 for v in latencies_s)
    return {"n": len(ms), "mean_ms": sum(ms) / len(ms), "p50_ms": _percentile(ms, 0.50),
            "p95_ms": _percentile(ms, 0.95), "max_ms": ms[-1]}


def _cpu_ticks(proc_stat):
    # The command name (field 2) is in parentheses and may contain spaces or
    # parentheses itself, so count fields from the last ')': state is field
    # 3, utime field 14, stime field 15.
    fields = proc_stat[proc_stat.rindex(")") + 2:].split()
    return int(fields[11]) + int(fields[12])


def cpu_percent(proc_stat_before, proc_stat_after, wall_s, clk_tck):
    """CPU use of one process between two /proc/<pid>/stat readings, in % of one core."""
    ticks = _cpu_ticks(proc_stat_after) - _cpu_ticks(proc_stat_before)
    return 100.0 * ticks / clk_tck / wall_s


def compare_detections(a, b):
    """Frame-by-frame difference between two lists of 5-float detections.

    Frames where `detected` differs are counted; the float difference is
    the largest over the other frames (a flip would otherwise swamp it).
    """
    if len(a) != len(b):
        raise ValueError(f"lengths differ: {len(a)} vs {len(b)}")
    flips = 0
    max_diff = 0.0
    for x, y in zip(a, b):
        if x[0] != y[0]:
            flips += 1
            continue
        max_diff = max([max_diff] + [abs(p - q) for p, q in zip(x, y)])
    return {"frames": len(a), "detected_differs": flips, "max_abs_diff": max_diff}


# --- capture --------------------------------------------------------------

def capture(args):
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("perception_bench_capture")
    frames = []
    meta = {}
    seen = [0]

    def on_image(msg):
        seen[0] += 1
        if len(frames) < args.count and (seen[0] - 1) % args.every == 0:
            meta.update(height=msg.height, width=msg.width, encoding=msg.encoding,
                        step=msg.step)
            frames.append(np.frombuffer(bytes(msg.data), dtype=np.uint8))

    node.create_subscription(Image, args.topic, on_image, qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    while len(frames) < args.count and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if len(frames) < args.count:
        print(f"capture: FAILED: {len(frames)}/{args.count} frames on {args.topic} "
              f"within {args.timeout} s")
        sys.exit(1)
    np.savez(args.out, frames=np.stack(frames), **meta)
    print(f"capture: {len(frames)} frames, {meta['width']}x{meta['height']} "
          f"{meta['encoding']} -> {args.out}")


# --- run ------------------------------------------------------------------

def _node_command(impl, backend):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    pkg = IMPLS[impl]
    # The executable itself, not `ros2 run`, so the pid is the node's.
    exe = Path(get_package_prefix(pkg)) / "lib" / pkg / "perception_node"
    params = {"image_topic": BENCH_IMAGE}
    if backend == "dnn":
        models = Path(get_package_share_directory("vision_bot")) / "models" / "dnn"
        params.update(detector_backend="dnn",
                      dnn_prototxt_path=str(models / "MobileNetSSD_deploy.prototxt"),
                      dnn_model_path=str(models / "MobileNetSSD_deploy.caffemodel"))
    cmd = [str(exe), "--ros-args", "-r", "__node:=perception_bench",
           "-r", f"/vision_bot/detection:={BENCH_DETECTION}",
           "-r", "/vision_bot/debug_image:=/bench/debug_image"]
    for name, value in params.items():
        cmd += ["-p", f"{name}:={value}"]
    return cmd


def _proc_stat(pid):
    return Path(f"/proc/{pid}/stat").read_text()


class Bench:
    """The publisher and detection subscriber shared by every node under test."""

    def __init__(self, messages):
        import rclpy
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float32MultiArray

        self.rclpy = rclpy
        self.node = rclpy.create_node("perception_bench")
        self.messages = messages
        self.pub = self.node.create_publisher(Image, BENCH_IMAGE, 10)
        self.received = []   # (perf_counter at callback, data)
        self.node.create_subscription(
            Float32MultiArray, BENCH_DETECTION,
            lambda msg: self.received.append((time.perf_counter(), list(msg.data))), 10)

    def spin(self, timeout=0.0):
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def wait_ready(self, proc, timeout=30.0):
        """Until the node answers a frame (it is subscribed and has warmed up)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and proc.poll() is None:
            if self.pub.get_subscription_count() > 0:
                self.received.clear()
                self.pub.publish(self.messages[0])
                end = time.monotonic() + 1.0
                while not self.received and time.monotonic() < end:
                    self.spin(0.01)
                if self.received:
                    for msg in self.messages[:10]:   # warm-up, not measured
                        self.lockstep(msg)
                    return
            self.spin(0.1)
        raise RuntimeError(f"node not ready within {timeout} s (exit code {proc.poll()})")

    def lockstep(self, msg, timeout=2.0):
        """Publish one frame, wait for one detection: (latency_s or None, data or None)."""
        # Drain anything late from a previous frame first.
        self.spin(0.0)
        self.received.clear()
        start = time.perf_counter()
        self.pub.publish(msg)
        while not self.received and time.perf_counter() - start < timeout:
            self.spin(0.01)
        if not self.received:
            return None, None
        stamp, data = self.received[0]
        return stamp - start, data

    def stream(self, pid, rate_hz, duration_s):
        """Publish frames at rate_hz; return (cpu %, detections/s out, frames/s in)."""
        period = 1.0 / rate_hz
        self.received.clear()
        before = _proc_stat(pid)
        start = time.perf_counter()
        sent = 0
        while time.perf_counter() - start < duration_s:
            if time.perf_counter() >= start + sent * period:
                self.pub.publish(self.messages[sent % len(self.messages)])
                sent += 1
            self.spin(0.001)
        wall = time.perf_counter() - start
        after = _proc_stat(pid)
        return (cpu_percent(before, after, wall, os.sysconf("SC_CLK_TCK")),
                len(self.received) / wall, sent / wall)


def _build_messages(path):
    import array

    import numpy as np
    from sensor_msgs.msg import Image

    data = np.load(path)
    messages = []
    for i, frame in enumerate(data["frames"]):
        msg = Image()
        msg.header.frame_id = "camera"
        msg.header.stamp.sec = i
        msg.height, msg.width = int(data["height"]), int(data["width"])
        msg.encoding, msg.step = str(data["encoding"]), int(data["step"])
        msg.data = array.array("B", frame.tobytes())
        messages.append(msg)
    return messages, f"{msg.width}x{msg.height} {msg.encoding}"


def _measure(bench, impl, args, rnd, rows):
    proc = subprocess.Popen(_node_command(impl, args.backend), stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    try:
        bench.wait_ready(proc)
        latencies, detections = [], []
        for i, msg in enumerate(bench.messages):
            latency, data = bench.lockstep(msg)
            detections.append(data)
            if latency is not None:
                latencies.append(latency)
            rows.append([rnd, impl, i, "" if latency is None else f"{latency * 1000:.3f}",
                         *(["" for _ in range(5)] if data is None else
                           [f"{v:.6f}" for v in data])])
        cpu, out_hz, in_hz = bench.stream(proc.pid, args.rate, args.stream_s)
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            log, _ = proc.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            log, _ = proc.communicate()
    up = [line for line in log.splitlines() if "perception_node up" in line]
    print(f"round {rnd} {impl}: {len(latencies)}/{len(bench.messages)} answered, "
          f"cpu {cpu:.1f}%, {out_hz:.1f} detections/s at {in_hz:.1f} Hz in"
          f" | {up[0].strip() if up else 'no startup line'}", flush=True)
    return latencies, detections, cpu, out_hz, in_hz


def run(args):
    import rclpy

    rclpy.init()
    messages, frame_desc = _build_messages(args.frames)
    bench = Bench(messages)
    results = {impl: {"latencies": [], "detections": [], "cpu": [], "out_hz": [], "in_hz": []}
               for impl in IMPLS}
    rows = []
    for rnd in range(args.rounds):
        order = list(IMPLS) if rnd % 2 == 0 else list(reversed(IMPLS))
        for impl in order:
            lat, det, cpu, out_hz, in_hz = _measure(bench, impl, args, rnd, rows)
            r = results[impl]
            r["latencies"] += lat
            r["detections"].append(det)
            r["cpu"].append(cpu)
            r["out_hz"].append(out_hz)
            r["in_hz"].append(in_hz)
    bench.node.destroy_node()
    rclpy.shutdown()

    # Parity on the first round, over frames both implementations answered.
    py, cc = results["python"]["detections"][0], results["cpp"]["detections"][0]
    both = [(a, b) for a, b in zip(py, cc) if a is not None and b is not None]
    parity = compare_detections([a for a, _ in both], [b for _, b in both])
    detected = sum(1 for a, _ in both if a[0] == 1.0)

    RESULTS.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = RESULTS / f"perception_impl_{args.backend}_{stamp}"
    with open(f"{base}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["round", "impl", "frame", "latency_ms", "detected", "offset_x", "offset_y",
                    "area_fraction", "confidence"])
        w.writerows(rows)

    table = ["| Implementation | answered | latency mean (ms) | p50 (ms) | p95 (ms) | max (ms) "
             "| CPU per round (% of one core) | detections/s while streaming |",
             "|---|---|---|---|---|---|---|---|"]
    for impl, r in results.items():
        s = summarize(r["latencies"])
        total = len(messages) * args.rounds
        table.append(
            f"| {impl} (`{IMPLS[impl]}`) | {s['n']}/{total} | {s['mean_ms']:.2f} "
            f"| {s['p50_ms']:.2f} | {s['p95_ms']:.2f} | {s['max_ms']:.2f} "
            f"| {', '.join(f'{c:.1f}' for c in r['cpu'])} "
            f"| {', '.join(f'{h:.1f}' for h in r['out_hz'])} |")
    in_hz = results["python"]["in_hz"] + results["cpp"]["in_hz"]
    cpu_model = next((line.split(":", 1)[1].strip() for line in
                      Path("/proc/cpuinfo").read_text().splitlines()
                      if line.startswith("model name")), "unknown")
    summary = "\n".join([
        f"# perception_node, Python vs C++ ({args.backend}), {stamp}",
        "",
        f"- Frames: {len(messages)} captured from the simulation camera ({frame_desc}),"
        f" file `{Path(args.frames).name}`.",
        f"- Configuration: `detector_backend={args.backend}`, every other parameter at its"
        " default; debug image on (the default). Each implementation started as its own"
        f" process, {args.rounds} rounds, order alternating (round 0: python first).",
        "- Latency: lockstep, publish one frame and wait for its detection, measured in the"
        " benchmark process from publish to the detection callback; includes the image's"
        " transport. 10 warm-up frames per start are not counted.",
        f"- CPU: the frames streamed at {args.rate:.0f} Hz for {args.stream_s:.0f} s"
        f" (achieved {min(in_hz):.1f} to {max(in_hz):.1f} Hz), the node process's"
        " utime+stime over wall time.",
        f"- Machine: {cpu_model}, {os.cpu_count()} logical CPUs visible to the container."
        + (f" {args.label}." if args.label else ""),
        f"- Code: commit `{git_commit()}`, `benchmarks/perception_impl.py`.",
        "",
        *table,
        "",
        f"Parity (round 0, {parity['frames']} frames both answered, {detected} with a"
        f" detection): `detected` differs on {parity['detected_differs']} frames; largest"
        f" difference in any value on the other frames {parity['max_abs_diff']:.2e}.",
        "",
        f"Raw data: `{base.name}.csv` (one row per frame per round per implementation).",
        "",
    ])
    Path(f"{base}.md").write_text(summary)
    print(summary)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    c = sub.add_parser("capture", help="record camera frames from a running simulation")
    c.add_argument("--out", required=True, help="output .npz")
    c.add_argument("--topic", default="/camera/image_raw")
    c.add_argument("--count", type=int, default=200)
    c.add_argument("--every", type=int, default=3,
                   help="keep every Nth frame, so the frames span more of the run")
    c.add_argument("--timeout", type=float, default=180.0, help="wall-clock seconds")
    r = sub.add_parser("run", help="measure both implementations on captured frames")
    r.add_argument("--frames", required=True, help=".npz written by capture")
    r.add_argument("--backend", choices=["hsv", "dnn"], default="hsv")
    r.add_argument("--rounds", type=int, default=3)
    r.add_argument("--rate", type=float, default=30.0, help="streaming rate, Hz")
    r.add_argument("--stream-s", type=float, default=20.0, help="streaming duration, seconds")
    r.add_argument("--label", default="", help="free-text note recorded in the summary")
    args = parser.parse_args()
    if args.command == "capture":
        capture(args)
    else:
        run(args)


if __name__ == "__main__":
    main()
