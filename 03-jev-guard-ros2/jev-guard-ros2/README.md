# jev-guard-ros2

A real-time safety and milestone watchdog for autonomous robot policies (LeRobot, π0, and other VLA deployments), built as a ROS 2 node around [Jev](https://openrouter.ai/blog/insights/what-is-jev/), TypeSafe's non-autoregressive decision model.

A VLA policy outputs action chunks, but something still has to check, many times a second, whether the robot is stalling, drifting, dropping the object, or about to hit something. Hand-written rules miss a lot of these cases. A chat LLM with JSON output is too slow to sit in the loop. `jev-guard` sends one batched Jev query per tick (default 10 Hz) and asks the model three kinds of typed question at the same time: **Noul** (a calibrated probability), **Choice** (a label with its distribution) and **Score** (a rubric score weighted by probability). Hard local limits sit underneath the model and never wait on the network.

> ⚠️ **This is a supervisory monitor, not a certified safety function.** It does not replace a hardware E-stop, a safety PLC, or the robot controller's own force and joint limits. Use it to add semantic failure detection on top of them.

---

## Architecture

```
┌─────────────────────┐
│  Robot Workcell     │ ──(up to 1 kHz joint states, F/T wrench)
│  (Cameras + Motors) │ ──(~20 Hz vision detections / VLA actions)
└──────────┬──────────┘
           │  ROS 2 topics
           ▼
┌────────────────────────────────────────────────────────────┐
│  jev_guard node                                            │
│                                                            │
│  50 Hz sampler ─► TelemetryWindow (rolling 1.5 s)          │
│                     │                                      │
│                     ├─► Local hard limits  (every tick,     │
│                     │     force / joint margin / staleness) │
│                     │                                      │
│                     └─► render_state() ─► ONE Jev request   │
│                           1. is_anomaly_detected   Noul     │
│                           2. policy_execution_state Choice  │
│                           3. grasp_stability        Score   │
│                           4. intervention_urgency   Score   │
│                                                            │
│  DecisionEngine: debounce, E-stop latch, degraded mode      │
└──────────┬─────────────────────────────────────────────────┘
           ▼
┌────────────────────────────────────────────────────────────┐
│  Hardware safety / teleop bridge                           │
│   anomaly p > 0.85 AND urgency ≥ 4  ─► /jev_guard/estop     │
│   bad state / low grasp / urgency≥4 ─► /jev_guard/teleop_request │
│   otherwise                          ─► continue autonomy   │
└────────────────────────────────────────────────────────────┘
```

### The questions (`jev_guard/schemas.py`)

| Name | Primitive | Output | Question |
|---|---|---|---|
| `is_anomaly_detected` | Noul | P(true) ∈ [0, 1] | Is the end-effector experiencing an unmodeled collision or stall? |
| `policy_execution_state` | Choice | label + distribution + confidence | `ON_TRACK`, `OBJECT_SLIPPED`, `KINEMATIC_SINGULARITY`, `STALLED`, `GOAL_REACHED` |
| `grasp_stability` | Score (1–5) | expected score | 1 = dropped … 5 = rigid / stable |
| `intervention_urgency` | Score (1–5) | expected score | 1 = none … 5 = stop immediately |

All four questions go in a single request, so they are judged against the same state and the round trip is paid once.

### Decision rules (`jev_guard/decision.py`)

| Layer | Condition | Action |
|---|---|---|
| Local | EE force > `max_ee_force_n` | **E-stop** |
| Local | joint-limit margin < `min_joint_limit_margin_rad` | **E-stop** |
| Local | telemetry older than `max_telemetry_age_s` | Teleop |
| Jev | anomaly p > 0.85 **and** urgency ≥ 4 | **E-stop** (immediate, latched) |
| Jev | urgency ≥ 4, or state ∈ {SLIPPED, SINGULARITY, STALLED} with p ≥ 0.6, or grasp < 2 while holding | Teleop (must hold for `handover_confirm_ticks`) |
| Degraded | Jev failed or timed out on N ticks in a row | Teleop (if `handover_on_jev_loss`) |

An E-stop stays latched until someone calls `ros2 service call /jev_guard/reset std_srvs/srv/Trigger`.

### Design notes

- **Numbers get words next to them.** Jev judges meaning. It is not built to do arithmetic on raw numbers. The renderer therefore writes each value with its reading, for example `measured speed 0.001 m/s (stationary)` or `38.2 N (a sudden spike, 9.4x baseline)`. The thresholds for those readings are in code, and unit tests cover them.
- **The query never blocks the executor.** Each tick picks up the previous query's result, if it has arrived, and sends a new one. Decisions therefore run about one tick behind the latest state. Results older than `max_result_age_s` are thrown away.
- **Trends are computed per phase.** Object drift, grip-force drop and goal progress only use samples from the current task phase. Without this, a normal phase change (APPROACH → LIFT) looks like a slip.
- **The client has no dependencies.** `jev_client.py` uses only `urllib`, so the package installs into any ROS 2 workspace without pip.

---

## Topics

| Direction | Topic | Type |
|---|---|---|
| sub | `/joint_states` | `sensor_msgs/JointState` |
| sub | `/ee_twist`, `/cmd_ee_twist` | `geometry_msgs/TwistStamped` (measured / commanded) |
| sub | `/ee_wrench` | `geometry_msgs/WrenchStamped` |
| sub | `/ee_pose`, `/goal_pose` | `geometry_msgs/PoseStamped` |
| sub | `/camera/detections` | `vision_msgs/Detection3DArray` |
| sub | `/task_phase` | `std_msgs/String` (`APPROACH`, `GRASP`, `LIFT`, `TRANSPORT`, `PLACE`, `RETREAT`) |
| pub | `/jev_guard/estop` | `std_msgs/Bool` |
| pub | `/jev_guard/teleop_request` | `std_msgs/Bool` |
| pub | `/jev_guard/status` | `std_msgs/String` (JSON: action, reasons, signals, latency p50/p95) |
| srv | `/jev_guard/reset` | `std_srvs/Trigger` |

Poses and detections must all be in the same frame, normally the robot base frame. Joint names, limits and thresholds are set in `config/guard.yaml`, which ships with a Franka Panda example.

---

## Quick start

```bash
# In a ROS 2 (Humble or newer) workspace
cd ~/ros2_ws/src && git clone https://github.com/imjbassi/jev-guard-ros2.git
cd ~/ros2_ws && rosdep install --from-paths src -y --ignore-src
colcon build --packages-select jev_guard && source install/setup.bash

export TYPESAFE_API_KEY=...            # from the TypeSafe developer console
ros2 launch jev_guard safety_monitor.launch.py

# No key? Run the pipeline end to end with the offline heuristic backend:
ros2 launch jev_guard safety_monitor.launch.py use_mock:=true
```

### Without ROS

The telemetry, client and decision logic are plain Python:

```bash
pip install -r requirements.txt
pytest -q                                                     # unit + replay tests
python3 -m jev_guard.replay tests/mock_robot_states.json --mock --show-state
TYPESAFE_API_KEY=... python3 -m jev_guard.replay tests/mock_robot_states.json
```

Replaying with the offline mock backend:

```
episode                          expected         got                delay
success_nominal                  CONTINUE         CONTINUE               -
collision_during_approach        ESTOP            ESTOP              0.12s
stall_during_approach            TELEOP_HANDOVER  TELEOP_HANDOVER    0.40s
slip_during_transport            TELEOP_HANDOVER  TELEOP_HANDOVER    0.32s
singularity_during_transport     TELEOP_HANDOVER  TELEOP_HANDOVER    0.60s
drop_during_lift                 TELEOP_HANDOVER  TELEOP_HANDOVER    0.12s
```

These numbers only check the pipeline plumbing. The mock backend is a set of hand-written rules, not a model. Run without `--mock` to see what Jev itself decides.

---

## Benchmark: Jev vs. GPT-4o JSON mode

`tests/test_latency.py` renders the same telemetry states for both backends, asks the same four questions, and runs every answer through the same validator.

```bash
export TYPESAFE_API_KEY=... OPENAI_API_KEY=...
python3 tests/test_latency.py --n 100 --openai-model gpt-4o
```

It prints p50/p95/p99 latency, the share of calls finished within 100 ms, error count, and cost per 1,000 calls. Raw numbers go to `benchmark_results.json`. Prices are CLI flags (`--jev-price-in`, `--openai-price-in`, …); check each provider's current pricing before you publish results.

| Backend | n | errors | p50 (ms) | p95 (ms) | p99 (ms) | ≤100 ms | $ / 1k calls |
|---|---:|---:|---:|---:|---:|---:|---:|
| Jev (jev-latest) | _run the benchmark_ | | | | | | |
| OpenAI gpt-4o (JSON mode) | _run the benchmark_ | | | | | | |

> Measured round-trip time depends on your network path to each API. For comparisons, run the benchmark from the machine that will host the robot.

---

## Test data

`tests/mock_robot_states.json` holds **synthetic** pick-and-place episodes made by `tests/generate_mock_states.py`: one clean run and five runs with one injected fault each (collision, stall, slip, singularity, drop). They exercise the pipeline and the thresholds. For real evaluation, export recorded rosbag telemetry into the same per-sample fields and replay that instead.

## Layout

```
jev-guard-ros2/
├── jev_guard/
│   ├── guard_node.py      # ROS 2 node: subscriptions, sampler, guard tick, publishers
│   ├── jev_client.py      # Batched System One client (+ offline MockJevClient)
│   ├── schemas.py         # Typed Noul / Choice / Score questions and answers
│   ├── telemetry.py       # Rolling window, features, state rendering
│   ├── decision.py        # Local limits + Jev thresholds, debounce, latch
│   └── replay.py          # Offline replay CLI
├── launch/safety_monitor.launch.py
├── config/guard.yaml
├── tests/
│   ├── test_client.py         # wire format, parsing, HTTP/timeout handling
│   ├── test_guard_logic.py    # telemetry, decisions, episode replay
│   ├── test_latency.py        # Jev vs. OpenAI benchmark
│   ├── generate_mock_states.py
│   └── mock_robot_states.json
├── package.xml / setup.py / setup.cfg
└── requirements.txt
```

## License

MIT
