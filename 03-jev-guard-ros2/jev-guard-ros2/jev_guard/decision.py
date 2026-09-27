"""Turn Jev answers + local telemetry into a safety verdict.

Two layers, evaluated every tick:

1. **Local hard limits** - deterministic checks on raw features (force
   ceiling, joint-limit margin, stale telemetry). They never wait on the
   network and always win.
2. **Jev judgement** - the batched Noul / Choice / Score answers, combined
   with explicit thresholds. E-stop fires immediately; a teleop handover must
   be confirmed on ``handover_confirm_ticks`` consecutive ticks to avoid
   flapping on a single noisy answer.

An E-stop latches until ``reset()`` is called (the ROS node exposes this as a
service), so a recovered-looking tick never silently resumes the robot.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional

from .schemas import Answer, ChoiceAnswer, NoulAnswer, ScoreAnswer
from .telemetry import HOLDING_PHASES, Features


class Action(enum.IntEnum):
    # Ordered by severity so ``max()`` picks the most conservative action.
    CONTINUE = 0
    TELEOP_HANDOVER = 1
    ESTOP = 2


@dataclass
class GuardConfig:
    # Jev thresholds
    anomaly_estop_prob: float = 0.85
    urgency_estop: float = 4.0
    urgency_handover: float = 4.0
    state_handover_prob: float = 0.6
    grasp_handover_below: float = 2.0
    handover_states: tuple = ("OBJECT_SLIPPED", "KINEMATIC_SINGULARITY", "STALLED")
    handover_confirm_ticks: int = 2
    # Local hard limits
    max_ee_force_n: float = 60.0
    min_joint_limit_margin_rad: float = 0.03
    max_telemetry_age_s: float = 0.25
    # Behaviour when Jev is unavailable
    max_consecutive_jev_failures: int = 3
    handover_on_jev_loss: bool = True


@dataclass
class Verdict:
    action: Action
    reasons: List[str] = field(default_factory=list)
    source: str = "none"  # "local", "jev", "jev+local", "degraded"
    signals: Dict[str, object] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, object]:
        return {
            "action": self.action.name,
            "reasons": self.reasons,
            "source": self.source,
            "signals": self.signals,
        }


def summarize_answers(answers: Mapping[str, Answer]) -> Dict[str, object]:
    out: Dict[str, object] = {}
    for name, a in answers.items():
        if isinstance(a, NoulAnswer):
            out[name] = round(a.probability, 3)
        elif isinstance(a, ChoiceAnswer):
            out[name] = {
                "label": a.label,
                "p": round(a.probabilities.get(a.label, a.confidence or 0.0), 3),
            }
        elif isinstance(a, ScoreAnswer):
            out[name] = round(a.score, 2)
    return out


class DecisionEngine:
    def __init__(self, config: Optional[GuardConfig] = None) -> None:
        self.cfg = config or GuardConfig()
        self._handover_streak = 0
        self._jev_failures = 0
        self._latched: Optional[Verdict] = None

    @property
    def latched(self) -> bool:
        return self._latched is not None

    def reset(self) -> None:
        self._latched = None
        self._handover_streak = 0

    def record_jev_failure(self) -> None:
        self._jev_failures += 1

    # ------------------------------------------------------------------ #
    def local_rules(self, f: Optional[Features]) -> Verdict:
        c = self.cfg
        if f is None:
            return Verdict(Action.TELEOP_HANDOVER, ["no telemetry received"], "local")
        reasons: List[str] = []
        action = Action.CONTINUE
        if f.telemetry_age_s > c.max_telemetry_age_s:
            action = max(action, Action.TELEOP_HANDOVER)
            reasons.append(f"telemetry stale ({f.telemetry_age_s * 1000:.0f} ms)")
        if f.ee_force > c.max_ee_force_n:
            action = max(action, Action.ESTOP)
            reasons.append(f"EE force {f.ee_force:.1f} N > limit {c.max_ee_force_n:.1f} N")
        if f.joint_limit_margin < c.min_joint_limit_margin_rad:
            action = max(action, Action.ESTOP)
            reasons.append(
                f"joint-limit margin {f.joint_limit_margin:.3f} rad < "
                f"{c.min_joint_limit_margin_rad:.3f} rad"
            )
        return Verdict(action, reasons, "local")

    def jev_rules(self, answers: Mapping[str, Answer], f: Features) -> Verdict:
        c = self.cfg
        reasons: List[str] = []
        action = Action.CONTINUE

        anomaly = answers.get("is_anomaly_detected")
        state = answers.get("policy_execution_state")
        grasp = answers.get("grasp_stability")
        urgency = answers.get("intervention_urgency")

        p = anomaly.probability if isinstance(anomaly, NoulAnswer) else 0.0
        u = urgency.score if isinstance(urgency, ScoreAnswer) else 1.0

        if p > c.anomaly_estop_prob and u >= c.urgency_estop:
            action = Action.ESTOP
            reasons.append(f"anomaly p={p:.2f} > {c.anomaly_estop_prob} and urgency {u:.1f}")
        else:
            if u >= c.urgency_handover:
                action = max(action, Action.TELEOP_HANDOVER)
                reasons.append(f"urgency {u:.1f} >= {c.urgency_handover}")
            if isinstance(state, ChoiceAnswer) and state.label in c.handover_states:
                sp = state.probabilities.get(state.label, state.confidence or 1.0)
                if sp >= c.state_handover_prob:
                    action = max(action, Action.TELEOP_HANDOVER)
                    reasons.append(f"policy state {state.label} (p={sp:.2f})")
            if (
                isinstance(grasp, ScoreAnswer)
                and f.phase in HOLDING_PHASES
                and grasp.score < c.grasp_handover_below
            ):
                action = max(action, Action.TELEOP_HANDOVER)
                reasons.append(f"grasp stability {grasp.score:.1f} during {f.phase}")

        return Verdict(action, reasons, "jev", summarize_answers(answers))

    # ------------------------------------------------------------------ #
    def decide(
        self, f: Optional[Features], answers: Optional[Mapping[str, Answer]] = None
    ) -> Verdict:
        """Combine layers. ``answers=None`` means no fresh Jev result this tick."""
        if self._latched is not None:
            return self._latched

        local = self.local_rules(f)
        if answers is not None and f is not None:
            self._jev_failures = 0
            jev = self.jev_rules(answers, f)
        else:
            jev = Verdict(Action.CONTINUE, [], "jev")

        # Debounce Jev-originated handovers.
        if jev.action == Action.TELEOP_HANDOVER:
            self._handover_streak += 1
            if self._handover_streak < self.cfg.handover_confirm_ticks:
                jev = Verdict(
                    Action.CONTINUE,
                    [f"pending handover ({self._handover_streak}/"
                     f"{self.cfg.handover_confirm_ticks}): " + "; ".join(jev.reasons)],
                    "jev",
                    jev.signals,
                )
        elif answers is not None:
            self._handover_streak = 0

        action = max(local.action, jev.action)
        reasons = local.reasons + jev.reasons
        source = (
            "jev+local" if local.action and jev.action
            else "local" if local.action
            else "jev"
        )

        if (
            answers is None
            and self._jev_failures >= self.cfg.max_consecutive_jev_failures
        ):
            source = "degraded"
            reasons.append(f"Jev unavailable for {self._jev_failures} consecutive ticks")
            if self.cfg.handover_on_jev_loss:
                action = max(action, Action.TELEOP_HANDOVER)

        verdict = Verdict(action, reasons, source, jev.signals)
        if action == Action.ESTOP:
            self._latched = verdict
        return verdict
