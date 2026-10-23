# perception_node, Python vs C++ (hsv), 20261022_200811

- Frames: 200 captured from the simulation camera (640x480 rgb8), file `frames_hsv.npz`.
- Configuration: `detector_backend=hsv`, every other parameter at its default; debug image on (the default). Each implementation started as its own process, 3 rounds, order alternating (round 0: python first).
- Latency: lockstep, publish one frame and wait for its detection, measured in the benchmark process from publish to the detection callback; includes the image's transport. 10 warm-up frames per start are not counted.
- CPU: the frames streamed at 30 Hz for 20 s (achieved 30.0 to 30.0 Hz), the node process's utime+stime over wall time.
- Machine: Intel(R) Core(TM) i5-1035G7 CPU @ 1.20GHz, 8 logical CPUs visible to the container. red-box world (track.world).
- Code: commit `b39f811`, `benchmarks/perception_impl.py`.

| Implementation | answered | latency mean (ms) | p50 (ms) | p95 (ms) | max (ms) | CPU per round (% of one core) | detections/s while streaming |
|---|---|---|---|---|---|---|---|
| python (`vision_bot`) | 600/600 | 22.43 | 20.59 | 37.70 | 147.52 | 44.4, 55.2, 56.5 | 30.0, 29.5, 30.0 |
| cpp (`vision_bot_perception`) | 600/600 | 11.77 | 12.21 | 19.63 | 77.36 | 27.8, 44.1, 45.3 | 30.0, 30.0, 29.8 |

Parity (round 0, 200 frames both answered, 200 with a detection): `detected` differs on 0 frames; largest difference in any value on the other frames 0.00e+00.

Raw data: `perception_impl_hsv_20261022_200811.csv` (one row per frame per round per implementation).
