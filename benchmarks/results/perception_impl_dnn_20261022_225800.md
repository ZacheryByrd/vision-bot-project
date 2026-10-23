# perception_node, Python vs C++ (dnn), 20261022_225800

- Frames: 200 captured from the simulation camera (640x480 rgb8), file `frames_dnn.npz`.
- Configuration: `detector_backend=dnn`, every other parameter at its default; debug image on (the default). Each implementation started as its own process, 3 rounds, order alternating (round 0: python first).
- Latency: lockstep, publish one frame and wait for its detection, measured in the benchmark process from publish to the detection callback; includes the image's transport. 10 warm-up frames per start are not counted.
- CPU: the frames streamed at 30 Hz for 20 s (achieved 28.9 to 30.0 Hz), the node process's utime+stime over wall time.
- Machine: Intel(R) Core(TM) i5-1035G7 CPU @ 1.20GHz, 8 logical CPUs visible to the container. person world (dnn_follow_launch.py).
- Code: commit `b39f811`, `benchmarks/perception_impl.py`.

| Implementation | answered | latency mean (ms) | p50 (ms) | p95 (ms) | max (ms) | CPU per round (% of one core) | detections/s while streaming |
|---|---|---|---|---|---|---|---|
| python (`vision_bot`) | 600/600 | 81.24 | 73.94 | 125.00 | 367.55 | 309.4, 418.7, 431.9 | 10.9, 18.1, 18.6 |
| cpp (`vision_bot_perception`) | 600/600 | 82.79 | 78.13 | 128.04 | 265.65 | 343.6, 290.3, 363.2 | 12.0, 11.8, 17.1 |

Parity (round 0, 200 frames both answered, 172 with a detection): `detected` differs on 0 frames; largest difference in any value on the other frames 0.00e+00.

Raw data: `perception_impl_dnn_20261022_225800.csv` (one row per frame per round per implementation).
