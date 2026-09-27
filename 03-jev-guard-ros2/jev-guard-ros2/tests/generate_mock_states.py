"""Generate tests/mock_robot_states.json.

The episodes are SYNTHETIC: parametric pick-and-place trajectories with a
single injected failure each, plus small Gaussian noise. They are meant to
exercise the pipeline and the decision thresholds, not to stand in for real
robot logs. Swap in recorded rosbag data (same fields) for real evaluation.

    python3 tests/generate_mock_states.py
"""

from __future__ import annotations

import json
import random
from pathlib import Path

HZ = 25
OUT = Path(__file__).with_name("mock_robot_states.json")

# (phase, duration_s, goal distance at start -> end in metres)
NOMINAL = [
    ("APPROACH", 1.2, 0.30, 0.02),
    ("GRASP", 0.6, 0.02, 0.005),
    ("LIFT", 0.6, 0.45, 0.40),
    ("TRANSPORT", 1.6, 0.40, 0.03),
    ("PLACE", 0.6, 0.03, 0.005),
]


def _base(rng: random.Random):
    t = 0.0
    for phase, dur, g0, g1 in NOMINAL:
        n = int(dur * HZ)
        for i in range(n):
            a = i / max(1, n - 1)
            holding = phase in ("LIFT", "TRANSPORT", "PLACE")
            moving = phase in ("APPROACH", "LIFT", "TRANSPORT")
            cmd = (0.12 if moving else 0.03) * (1 - 0.7 * a if phase != "LIFT" else 1)
            yield {
                "t": round(t, 3),
                "phase": phase,
                "cmd_ee_speed": cmd,
                "ee_speed": cmd * (0.95 + 0.05 * rng.random()),
                "ee_force": 4.0 + rng.gauss(0, 0.4) + (3.0 if holding else 0.0),
                "gripper_width": 0.04 if holding or phase == "GRASP" and a > 0.5 else 0.08,
                "gripper_force": 20.0 + rng.gauss(0, 0.5) if holding else 0.0,
                "object_visible": True,
                "object_offset": 0.012 + rng.gauss(0, 0.001) if holding else g0 + (g1 - g0) * a,
                "goal_distance": g0 + (g1 - g0) * a,
                "joint_speed_max": 0.4 + 0.2 * rng.random(),
                "joint_limit_margin": 0.6 - 0.1 * a,
            }
            t += 1.0 / HZ


def _round(s):
    return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in s.items()}


def episode(name, description, expected, event_t, mutate, seed):
    rng = random.Random(seed)
    samples = []
    for s in _base(rng):
        if event_t is not None and s["t"] >= event_t:
            mutate(s, s["t"] - event_t, rng)
        samples.append(_round(s))
    return {
        "name": name,
        "description": description,
        "synthetic": True,
        "expected_action": expected,
        "event_t": event_t,
        "samples": samples,
    }


def collision(s, dt, rng):
    s["ee_speed"] = 0.001 + rng.random() * 0.001
    s["ee_force"] = 38.0 + rng.gauss(0, 2.0)


def stall(s, dt, rng):
    s["ee_speed"] = 0.002 * rng.random()
    s["goal_distance"] = s["object_offset"] = 0.155  # frozen where the stall began
    s["ee_force"] = 5.0 + rng.gauss(0, 0.4)


def slip(s, dt, rng):
    s["object_offset"] = 0.012 + 0.08 * min(dt, 0.8)
    s["gripper_force"] = max(2.0, 20.0 - 30.0 * dt)
    s["gripper_width"] = 0.035


def singularity(s, dt, rng):
    s["joint_limit_margin"] = max(0.05, 0.5 - 1.0 * dt)
    s["joint_speed_max"] = 0.5 + 2.5 * min(dt, 0.6)


def drop(s, dt, rng):
    s["object_visible"] = False
    s["object_offset"] = None
    s["goal_distance"] = None
    s["gripper_force"] = 0.0
    s["gripper_width"] = 0.0


def build():
    return [
        episode("success_nominal", "Clean pick-and-place, no fault.", "CONTINUE", None, None, 1),
        episode(
            "collision_during_approach",
            "Arm hits an unmodeled obstacle mid-approach: force spike and stop.",
            "ESTOP", 0.8, collision, 2,
        ),
        episode(
            "stall_during_approach",
            "Policy keeps commanding motion but the arm does not move (no contact force).",
            "TELEOP_HANDOVER", 0.6, stall, 3,
        ),
        episode(
            "slip_during_transport",
            "Object slides out of the fingers while being carried.",
            "TELEOP_HANDOVER", 3.0, slip, 4,
        ),
        episode(
            "singularity_during_transport",
            "Arm drives toward a joint limit; joint speeds spike.",
            "TELEOP_HANDOVER", 2.8, singularity, 5,
        ),
        episode(
            "drop_during_lift",
            "Object falls out of the gripper right after lift-off.",
            "TELEOP_HANDOVER", 2.0, drop, 6,
        ),
    ]


if __name__ == "__main__":
    eps = build()
    OUT.write_text(json.dumps({"hz": HZ, "episodes": eps}, separators=(",", ":")) + "\n")
    print(f"wrote {OUT} ({sum(len(e['samples']) for e in eps)} samples, {len(eps)} episodes)")
