# 03 — jev-guard: A Safety Watchdog for Robot Policies

A VLA policy (π0, openpi, LeRobot) turns camera frames into motor commands. It has
no idea when it's failing. It will keep pushing into an obstacle, keep "carrying" an
object it dropped, or drive the arm into a joint limit. Something outside the
policy has to watch, many times a second, and decide: **continue**, **hand over to a
human**, or **stop**.

[`jev-guard-ros2/`](jev-guard-ros2/) is that watchdog, built as a ROS 2 node. It's a
snapshot of [imjbassi/jev-guard-ros2](https://github.com/imjbassi/jev-guard-ros2)
(commit `9f15343`) with one bug fix (see [Bug found while studying](#bug-found-while-studying)).
Its own [README](jev-guard-ros2/README.md) covers the API, topics and benchmark. This page
covers **the ideas worth learning** from it, plus a fake workcell so you can watch it work.

```
 fake_workcell.py ──(8 sensor topics, 50 Hz)──▶ jev_guard ──▶ /jev_guard/estop          (Bool, 10 Hz)
        ▲                                                   ──▶ /jev_guard/teleop_request (Bool)
        └──────────── freezes the arm on estop ◀──────────── ──▶ /jev_guard/status        (JSON)
```

---

## Concepts

### 1. Layered safety: the monitor is not the safety system

Stack of protections, bottom one wins:

| Layer | Example | Speed | Depends on |
|---|---|---|---|
| Hardware | E-stop button, safety PLC, motor current limits | µs | nothing |
| Controller | joint/force limits in the robot driver | ~1 kHz | the controller |
| **Local rules** in the guard | force > 60 N, joint margin < 0.03 rad | every tick | only local data |
| **Model judgement** in the guard | "is the object slipping?" | ~10 Hz | network + model |

jev-guard only covers the bottom two rows of this table, the local rules and the model judgement. Its README says it plainly: *supervisory monitor, not a certified
safety function*. Within the guard the same idea repeats: `decision.py` evaluates
**local hard limits every tick, without the network**, and they always override the model.

### 2. Never block the control loop

Calling a model over the network takes about 80–250 ms. Doing that inside a 10 Hz timer
callback would stall the whole node. `guard_node._tick` **pipelines** instead:

```
tick k:   collect answer from tick k-1 (if it arrived)  →  submit new query  →  decide
```

The query runs on a thread pool (`JevClient.submit` returns a `Future`). Decisions run one tick
behind the newest state, and answers older than `max_result_age_s` are thrown away.
**Stale data is treated as no data.**

### 3. Windowing: turn 1 kHz streams into a few numbers

`telemetry.py` keeps a rolling **1.5 s window** of fused samples (sampled at 50 Hz from the
latest value of each topic) and reduces it to `Features`: stall duration, force spike ratio
(peak ÷ median), grip-force drop, object drift relative to the gripper, and progress toward the goal.

Two details worth copying:
- **Median as the baseline**: the force spike doesn't pull its own baseline up.
- **Trends are computed per phase**: going from GRASP to LIFT changes what the goal means. Computed across phases, a normal transition would look like a slip.

### 4. Give a language model words, not just numbers

`render_state()` writes each number together with what it means:
`measured speed 0.000 m/s (stationary)` and `38.2 N (a sudden spike, 9.4x baseline)`. The
thresholds behind those readings are in code and have unit tests. The model only judges meaning and never does arithmetic.

### 5. Typed questions instead of free text

One batched request asks four questions, each with a typed answer (`schemas.py`):

| Question | Type | Answer |
|---|---|---|
| `is_anomaly_detected` | Noul | probability 0–1 |
| `policy_execution_state` | Choice | one of ON_TRACK / OBJECT_SLIPPED / KINEMATIC_SINGULARITY / STALLED / GOAL_REACHED |
| `grasp_stability` | Score 1–5 | expected score |
| `intervention_urgency` | Score 1–5 | expected score |

Typed answers can be **validated** and compared to thresholds. Free text can't.

### 6. Decision logic: severity, debounce, latch, degraded mode

`decision.py`, `DecisionEngine`:

- **Severity ordering**: `Action` is an `IntEnum` (CONTINUE < TELEOP < ESTOP), so combining layers is just `max()`. The most conservative answer wins.
- **Debounce**: a handover must hold for 2 ticks in a row. One noisy answer shouldn't pull a human in.
- **E-stop fires immediately and latches**: it stays on until someone calls `/jev_guard/reset`. A tick that looks recovered never quietly restarts the robot.
- **Degraded mode**: if the model fails 3 ticks in a row, request a handover.

### 7. ROS 2 wiring choices (ties to topic 02)

- Sensor subscriptions use `qos_profile_sensor_data`, which is best effort, so they accept both best-effort and reliable publishers (topic 02, exercise 1).
- The `estop` and `teleop_request` publishers are **reliable**. You don't want to lose a stop command.
- `estop` is published **every tick**, not only when it changes. That turns it into a **heartbeat**, which matters a lot. See the next section.

---

## Run it

```bash
./run_demo.sh --docker                    # clean runs; builds in ros:humble, no API key needed
./run_demo.sh --docker --fault collision  # or: stall | slip | singularity
```

This builds the package, launches the guard with `use_mock:=true`, starts the fake workcell,
and opens a shell. The **mock backend** uses hand-written rules in place of the model, so you
can study the plumbing offline. With a `TYPESAFE_API_KEY` you can drop `use_mock` and get
real model answers.

### What each fault does (real runs)

| `--fault` | What the workcell does | Guard's reaction |
|---|---|---|
| none | full pick-and-place loop | `CONTINUE` throughout, logs each phase change |
| `collision` | arm stops mid-approach, 35 N contact force | `TELEOP_HANDOVER: urgency 4.6` then `ESTOP: anomaly p=0.90 > 0.85 and urgency 4.6`, latched, and the arm freezes |
| `stall` | policy commands motion, arm doesn't move | `TELEOP_HANDOVER: policy state STALLED (p=0.80)` |
| `slip` | object slides out during TRANSPORT, grip force drains | `TELEOP_HANDOVER: policy state OBJECT_SLIPPED (p=0.80)` |
| `singularity` | joint 4 hits 0.06 rad from its limit, joint speed 2.5 rad/s | `TELEOP_HANDOVER: urgency 4.0 >= 4.0; policy state KINEMATIC_SINGULARITY (p=0.80)` |

Each fault lasts 3 s, then the workcell starts a new run and the guard goes back to `CONTINUE`.

### Inspect it with the topic 02 tools

```
$ ros2 node info /jev_guard
/jev_guard
  Subscribers:
    /camera/detections: vision_msgs/msg/Detection3DArray
    /cmd_ee_twist: geometry_msgs/msg/TwistStamped
    /ee_pose: geometry_msgs/msg/PoseStamped
    /ee_twist: geometry_msgs/msg/TwistStamped
    /ee_wrench: geometry_msgs/msg/WrenchStamped
    /goal_pose: geometry_msgs/msg/PoseStamped
    /joint_states: sensor_msgs/msg/JointState
    /task_phase: std_msgs/msg/String
  Publishers:
    /jev_guard/estop: std_msgs/msg/Bool
    /jev_guard/status: std_msgs/msg/String
    /jev_guard/teleop_request: std_msgs/msg/Bool
    ...
  Service Servers:
    /jev_guard/reset: std_srvs/srv/Trigger
    ...

$ ros2 topic hz /jev_guard/status
average rate: 9.998
	min: 0.099s max: 0.101s std dev: 0.00025s window: 21

$ ros2 topic echo /jev_guard/status --once --field data      # during --fault collision
{"action": "ESTOP", "reasons": ["anomaly p=0.90 > 0.85 and urgency 4.6"], "source": "jev",
 "signals": {"is_anomaly_detected": 0.9, "policy_execution_state": {"label": "STALLED", "p": 0.8},
 "grasp_stability": 3.0, "intervention_urgency": 4.6}, "phase": "APPROACH", "latched": true,
 "jev_latency_p50_ms": 80.2, "jev_latency_p95_ms": 80.4}

$ ros2 service call /jev_guard/reset std_srvs/srv/Trigger
[jev_guard]: reset requested: E-stop latch cleared
[fake_workcell]: E-stop cleared: resuming
[jev_guard]: ESTOP [jev]: anomaly p=0.90 > 0.85 and urgency 4.6    <- obstacle still there: trips again
```

That last line is the latch doing its job. Resetting doesn't clear the cause of the fault.

### Without ROS

The logic is plain Python, so you can replay recorded episodes offline:

```bash
cd jev-guard-ros2
pip install pytest && python3 -m pytest -q        # 27 passed, 2 skipped
python3 -m jev_guard.replay tests/mock_robot_states.json --mock --show-state
```

---

## Bug found while studying

With `--fault collision` the guard **crashed on its first E-stop**:

```
[jev_guard]: TELEOP_HANDOVER [jev]: urgency 4.6 >= 4.0
ValueError: Logger severity cannot be changed between calls.
[ERROR] [guard_node-1]: process has died
```

`_tick` picked the logger method dynamically (`log = get_logger().error if ESTOP else get_logger().warn`)
and called it from a single line. rclpy ties each logging **call site** to a single severity, so the
first `warn`-then-`error` sequence raised an exception. The unit tests and replay never go through the ROS logger, so they didn't
catch it. It only showed up when running the real node. Fixed here by using two separate calls in
[`guard_node.py`](jev-guard-ros2/jev_guard/guard_node.py). The upstream repo still has the bug.

**The lesson is bigger than the bug.** The E-stop went out once, and then the monitor died. Any
consumer that reacts only to `estop == true` would treat the silence afterwards as "all clear". That's
why [`fake_workcell.py`](fake_workcell.py) is **fail-safe**: it only moves while it's getting the
guard's heartbeat, and it freezes if `/jev_guard/estop` goes quiet for 0.5 s. Tested by killing the guard mid-run:

```
[fake_workcell]: run 1: TRANSPORT
[fake_workcell]: no guard heartbeat for 0.5 s: arm frozen
```

A watchdog needs a watchdog. Real systems use the same pattern: safety PLCs require a
periodic signal, and missing it means stop.

---

## Exercises

1. **Local limit vs model.** Run `--fault collision` with `max_ee_force_n:=30.0` (edit `config/guard.yaml` or pass a params file). Which layer fires first now, and what does `source` say in the status?
2. **Debounce.** Set `handover_confirm_ticks` to 5 and run `--fault stall`. How much later does the handover arrive? Why is 1 a bad value?
3. **Degraded mode.** Launch without `use_mock` and without an API key. What happens? Now read `JevClient.__init__`. Should a missing key crash the node or put it in degraded mode?
4. **Stale telemetry.** Kill `fake_workcell.py` while the guard is running. Which rule fires, and after how long?
5. **Add a fault.** Add `--fault drop` to the workcell: the object disappears from `/camera/detections` during LIFT. Check your prediction against `heuristic_answers()` in `telemetry.py`.

## Check yourself

1. Why are the local hard limits checked without waiting for the model's answer?
2. What would go wrong if `_tick` called the model synchronously?
3. Why is the force baseline a median and not a mean?
4. What's the difference between how TELEOP and ESTOP are triggered and cleared?
5. Why is publishing `estop=false` every tick safer than only publishing when it changes?
