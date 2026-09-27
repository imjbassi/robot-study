# Robot Study

Notes and small, runnable examples for studying robotics. Each topic has its own
folder with a `README.md` explaining the concept and some simple code to poke at.

## Topics

| # | Folder | What it covers |
|---|--------|----------------|
| 01 | [`01-docker-for-ros2`](01-docker-for-ros2/) | The `docker run` flags a real robot container needs (`--network host`, `--ipc host`, `--gpus all`, `-v /dev:/dev`, `--privileged`) and why |
| 02 | [`02-ros2-cli-introspection`](02-ros2-cli-introspection/) | Debugging a running ROS 2 system with `ros2 topic list/hz/echo`, `ros2 node info`, `ros2 doctor`; QoS and domain-ID exercises |
| 03 | [`03-jev-guard-ros2`](03-jev-guard-ros2/) | A safety watchdog for VLA policies: layered safety, non-blocking model queries, debounce/latch, fail-safe heartbeats; with a fake workcell and fault injection |

## Adding a topic

1. Create a folder named `NN-topic-name/`.
2. Add a `README.md` with the concept in your own words.
3. Add small, self-contained code that demonstrates it (standard library where possible).
4. Add a row to the table above.
