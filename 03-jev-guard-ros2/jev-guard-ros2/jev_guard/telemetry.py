"""Telemetry windowing and state rendering.

High-rate sensor streams (joint states at up to 1 kHz, vision at ~20 Hz) are
folded into a short rolling window. Every guard tick the window is reduced to
a small set of ``Features`` and rendered as a compact text ``state`` for Jev.

Jev is a semantic judge, not a calculator, so the renderer states numbers
*and* the qualitative reading of each number ("0.002 m/s - effectively
stationary"). The thresholds that produce those readings live here, in code,
where they can be unit tested.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
from typing import Deque, Dict, Mapping, Optional

from .schemas import (
    Answer,
    ChoiceAnswer,
    ChoiceQuestion,
    NoulAnswer,
    NoulQuestion,
    Question,
    ScoreAnswer,
    ScoreQuestion,
)

PHASES = ("APPROACH", "GRASP", "LIFT", "TRANSPORT", "PLACE", "RETREAT")
HOLDING_PHASES = ("LIFT", "TRANSPORT", "PLACE")

# Qualitative bands used both in the rendered text and in the mock heuristics.
MOVING_SPEED = 0.01  # m/s; below this the end-effector is "stationary"
COMMANDED_SPEED = 0.02  # m/s; above this the policy is "asking for motion"
FORCE_SPIKE_RATIO = 2.5  # peak / baseline external force
SLIP_OFFSET = 0.015  # m; object centroid drift relative to the gripper
LOW_JOINT_MARGIN = 0.10  # rad from the nearest joint limit
HIGH_JOINT_SPEED = 2.0  # rad/s


@dataclass(frozen=True)
class Sample:
    """One fused telemetry sample. Units: SI (m, m/s, N, rad, rad/s, s)."""

    t: float
    phase: str
    cmd_ee_speed: float
    ee_speed: float
    ee_force: float
    gripper_width: float
    gripper_force: float
    object_visible: bool
    object_offset: Optional[float]  # object centroid distance from the gripper frame
    goal_distance: Optional[float]  # object (or EE) distance to the phase goal
    joint_speed_max: float
    joint_limit_margin: float


@dataclass(frozen=True)
class Features:
    phase: str
    window_s: float
    cmd_ee_speed: float
    ee_speed: float
    stall_duration_s: float
    ee_force: float
    ee_force_baseline: float
    force_spike_ratio: float
    gripper_width: float
    gripper_force: float
    gripper_force_drop: float  # fraction lost vs. window max, 0..1
    object_visible: bool
    object_offset: Optional[float]
    object_offset_drift: Optional[float]  # change over the window
    goal_distance: Optional[float]
    goal_progress: Optional[float]  # metres closer to goal over the window (+ = closer)
    joint_speed_max: float
    joint_limit_margin: float
    telemetry_age_s: float

    def as_dict(self) -> Dict:
        return asdict(self)


class TelemetryWindow:
    """Rolling window of fused samples, trimmed by age."""

    def __init__(self, window_s: float = 1.5, maxlen: int = 4096) -> None:
        self.window_s = window_s
        self._buf: Deque[Sample] = deque(maxlen=maxlen)

    def __len__(self) -> int:
        return len(self._buf)

    def add(self, s: Sample) -> None:
        if self._buf and s.t < self._buf[-1].t:
            # Out-of-order sample (e.g. clock jump); restart the window.
            self._buf.clear()
        self._buf.append(s)
        cutoff = s.t - self.window_s
        while self._buf and self._buf[0].t < cutoff:
            self._buf.popleft()

    def features(self, now: Optional[float] = None) -> Optional[Features]:
        if not self._buf:
            return None
        buf = list(self._buf)
        last = buf[-1]
        now = last.t if now is None else now

        # Stall: how long, counting back from the newest sample, the policy has
        # commanded motion while the end-effector stayed stationary.
        stall = 0.0
        for prev, cur in zip(reversed(buf[:-1]), reversed(buf)):
            if cur.cmd_ee_speed > COMMANDED_SPEED and cur.ee_speed < MOVING_SPEED:
                stall += cur.t - prev.t
            else:
                break

        forces = sorted(s.ee_force for s in buf)
        baseline = forces[len(forces) // 2]  # median is robust to the spike itself
        peak = max(s.ee_force for s in buf[-max(1, len(buf) // 5):])
        spike = peak / max(baseline, 1.0)

        # Grip, object and goal trends only make sense within one phase: the
        # "goal" changes meaning at a phase boundary and the grip is only
        # established during GRASP.
        cur_phase = [s for s in buf if s.phase == last.phase]

        gmax = max(s.gripper_force for s in cur_phase)
        g_drop = (gmax - last.gripper_force) / gmax if gmax > 1e-6 else 0.0

        offsets = [s.object_offset for s in cur_phase if s.object_offset is not None]
        drift = (offsets[-1] - offsets[0]) if len(offsets) >= 2 else None

        goals = [s.goal_distance for s in cur_phase if s.goal_distance is not None]
        progress = (goals[0] - goals[-1]) if len(goals) >= 2 else None

        return Features(
            phase=last.phase,
            window_s=buf[-1].t - buf[0].t,
            cmd_ee_speed=last.cmd_ee_speed,
            ee_speed=last.ee_speed,
            stall_duration_s=stall,
            ee_force=last.ee_force,
            ee_force_baseline=baseline,
            force_spike_ratio=spike,
            gripper_width=last.gripper_width,
            gripper_force=last.gripper_force,
            gripper_force_drop=max(0.0, g_drop),
            object_visible=last.object_visible,
            object_offset=last.object_offset,
            object_offset_drift=drift,
            goal_distance=last.goal_distance,
            goal_progress=progress,
            joint_speed_max=max(s.joint_speed_max for s in buf),
            joint_limit_margin=min(s.joint_limit_margin for s in buf),
            telemetry_age_s=max(0.0, now - last.t),
        )


# --------------------------------------------------------------------------- #
# Rendering
# --------------------------------------------------------------------------- #
def _speed_word(v: float) -> str:
    return "stationary" if v < MOVING_SPEED else ("slow" if v < 0.05 else "moving")


def render_state(f: Features) -> str:
    """Render features as the text ``state`` Jev evaluates."""
    lines = [
        f"Task: pick-and-place. Current phase: {f.phase}.",
        f"Telemetry window: last {f.window_s:.2f} s.",
        (
            f"End-effector: commanded speed {f.cmd_ee_speed:.3f} m/s "
            f"({_speed_word(f.cmd_ee_speed)}), measured speed {f.ee_speed:.3f} m/s "
            f"({_speed_word(f.ee_speed)})."
        ),
    ]
    if f.stall_duration_s > 0:
        lines.append(
            f"Motion has been commanded but the end-effector has not moved for "
            f"{f.stall_duration_s:.2f} s."
        )
    spike = "a sudden spike" if f.force_spike_ratio >= FORCE_SPIKE_RATIO else "steady"
    lines.append(
        f"External force at end-effector: {f.ee_force:.1f} N, baseline {f.ee_force_baseline:.1f} N "
        f"({spike}, {f.force_spike_ratio:.1f}x baseline)."
    )
    lines.append(
        f"Gripper: width {f.gripper_width * 1000:.0f} mm, grip force {f.gripper_force:.1f} N"
        + (
            f", down {f.gripper_force_drop * 100:.0f}% from its peak in this window."
            if f.gripper_force_drop >= 0.2
            else ", stable."
        )
    )
    if not f.object_visible:
        lines.append("Vision: target object NOT detected by the camera.")
    elif f.object_offset is not None:
        drift = ""
        if f.object_offset_drift is not None and f.phase in HOLDING_PHASES:
            moving = abs(f.object_offset_drift) >= SLIP_OFFSET
            drift = (
                f"; it moved {f.object_offset_drift * 1000:+.0f} mm relative to the gripper "
                f"({'shifting in the grasp' if moving else 'fixed in the grasp'})"
            )
        lines.append(
            f"Vision: object centroid {f.object_offset * 1000:.0f} mm from the gripper{drift}."
        )
    if f.goal_distance is not None:
        prog = ""
        if f.goal_progress is not None:
            if f.goal_progress > 0.005:
                prog = f", {f.goal_progress * 1000:.0f} mm closer than at window start"
            elif f.goal_progress < -0.005:
                prog = f", {-f.goal_progress * 1000:.0f} mm FARTHER than at window start"
            else:
                prog = ", no progress over the window"
        lines.append(f"Distance to phase goal: {f.goal_distance * 1000:.0f} mm{prog}.")
    lines.append(
        f"Arm: max joint speed {f.joint_speed_max:.2f} rad/s"
        + (" (unusually high)" if f.joint_speed_max >= HIGH_JOINT_SPEED else "")
        + f", closest joint-limit margin {f.joint_limit_margin:.2f} rad"
        + (" (near a limit)." if f.joint_limit_margin < LOW_JOINT_MARGIN else ".")
    )
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# Offline heuristics (used by MockJevClient only)
# --------------------------------------------------------------------------- #
def heuristic_answers(f: Features, questions: Mapping[str, Question]) -> Dict[str, Answer]:
    """Hand-written stand-in for the model, keyed by question name.

    Deterministic and intentionally simple; good enough to drive tests and a
    no-network demo, not a substitute for the real model.
    """
    stalled = f.stall_duration_s >= 0.3
    collided = f.force_spike_ratio >= FORCE_SPIKE_RATIO
    holding = f.phase in HOLDING_PHASES
    slipping = holding and (
        (f.object_offset_drift is not None and abs(f.object_offset_drift) >= SLIP_OFFSET)
        or f.gripper_force_drop >= 0.4
        or not f.object_visible
    )
    singular = f.joint_limit_margin < LOW_JOINT_MARGIN or f.joint_speed_max >= HIGH_JOINT_SPEED
    done = f.goal_distance is not None and f.goal_distance < 0.01 and not (stalled or collided)

    if collided or stalled:
        p_anom = 0.9 if (collided and stalled) else 0.8
    else:
        p_anom = 0.05

    state = (
        "OBJECT_SLIPPED" if slipping
        else "KINEMATIC_SINGULARITY" if singular
        else "STALLED" if stalled
        else "GOAL_REACHED" if done
        else "ON_TRACK"
    )

    if not holding:
        grasp = 3.0
    elif not f.object_visible or f.gripper_force < 1.0:
        grasp = 1.1
    elif slipping:
        grasp = 2.0
    else:
        grasp = 4.6

    urgency = 1.2
    if collided:
        urgency = 4.6
    elif stalled or singular or slipping:
        urgency = 4.0 if (slipping and grasp < 1.5) or singular else 3.4

    out: Dict[str, Answer] = {}
    for name, q in questions.items():
        if isinstance(q, NoulQuestion) and name == "is_anomaly_detected":
            out[name] = NoulAnswer(probability=p_anom)
        elif isinstance(q, ChoiceQuestion) and name == "policy_execution_state":
            label = state if state in q.options else q.labels[0]
            probs = {k: (0.8 if k == label else 0.2 / (len(q.options) - 1)) for k in q.labels}
            out[name] = ChoiceAnswer(label=label, probabilities=probs, confidence=0.8)
        elif isinstance(q, ScoreQuestion) and name == "grasp_stability":
            out[name] = ScoreAnswer(score=min(max(grasp, q.min_level), q.max_level))
        elif isinstance(q, ScoreQuestion) and name == "intervention_urgency":
            out[name] = ScoreAnswer(score=min(max(urgency, q.min_level), q.max_level))
    return out
