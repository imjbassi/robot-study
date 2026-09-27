"""Typed definitions for the three Jev question primitives and their answers.

Jev (TypeSafe's System One model) answers three kinds of questions about a
shared text ``state``:

* ``Noul``   - a yes/no proposition -> calibrated probability in [0, 1]
* ``Choice`` - pick one label from a fixed set -> label, distribution, confidence
* ``Score``  - rate against ordered, described levels -> probability-weighted score

Questions are plain dataclasses so they can be declared once, validated at
import time, and serialised into a single batched request.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Mapping, Optional, Tuple, Union

MAX_CHOICE_OPTIONS = 255
MIN_SCORE_LEVELS = 2
MAX_SCORE_LEVELS = 10


# --------------------------------------------------------------------------- #
# Questions
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class NoulQuestion:
    """A crisp yes/no proposition. The answer is P(proposition is true)."""

    instructions: str

    def to_wire(self) -> dict:
        return {"type": "noul", "instructions": self.instructions}


@dataclass(frozen=True)
class ChoiceQuestion:
    """Select exactly one option. ``options`` maps label -> description."""

    instructions: str
    options: Mapping[str, str]

    def __post_init__(self) -> None:
        if len(self.options) < 2:
            raise ValueError("ChoiceQuestion needs at least 2 options")
        if len(self.options) > MAX_CHOICE_OPTIONS:
            raise ValueError(f"ChoiceQuestion supports at most {MAX_CHOICE_OPTIONS} options")

    @property
    def labels(self) -> Tuple[str, ...]:
        return tuple(self.options.keys())

    def to_wire(self) -> dict:
        return {
            "type": "choice",
            "instructions": self.instructions,
            "criteria": dict(self.options),
        }


@dataclass(frozen=True)
class ScoreQuestion:
    """Rate the state against ordered levels.

    ``levels`` is an ordered mapping of ``level_value -> description``. The
    level values are the numbers the expected score is computed over, so a
    1..5 rubric is written ``{1: "...", 2: "...", ..., 5: "..."}``.
    """

    instructions: str
    levels: Mapping[int, str]

    def __post_init__(self) -> None:
        n = len(self.levels)
        if not MIN_SCORE_LEVELS <= n <= MAX_SCORE_LEVELS:
            raise ValueError(
                f"ScoreQuestion needs {MIN_SCORE_LEVELS}-{MAX_SCORE_LEVELS} levels, got {n}"
            )
        keys = list(self.levels.keys())
        if keys != sorted(keys):
            raise ValueError("ScoreQuestion levels must be in ascending order")

    @property
    def min_level(self) -> int:
        return min(self.levels)

    @property
    def max_level(self) -> int:
        return max(self.levels)

    def to_wire(self) -> dict:
        return {
            "type": "score",
            "instructions": self.instructions,
            "criteria": {str(k): v for k, v in self.levels.items()},
        }


Question = Union[NoulQuestion, ChoiceQuestion, ScoreQuestion]


# --------------------------------------------------------------------------- #
# Answers
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class NoulAnswer:
    probability: float


@dataclass(frozen=True)
class ChoiceAnswer:
    label: str
    probabilities: Dict[str, float] = field(default_factory=dict)
    confidence: Optional[float] = None


@dataclass(frozen=True)
class ScoreAnswer:
    score: float
    level_probabilities: Dict[str, float] = field(default_factory=dict)
    confidence: Optional[float] = None


Answer = Union[NoulAnswer, ChoiceAnswer, ScoreAnswer]


# --------------------------------------------------------------------------- #
# The guard's question set
# --------------------------------------------------------------------------- #
POLICY_STATES: Dict[str, str] = {
    "ON_TRACK": "The policy is making normal progress toward the goal of the current phase.",
    "OBJECT_SLIPPED": "The grasped object is moving relative to the gripper or has been dropped.",
    "KINEMATIC_SINGULARITY": (
        "The arm is near a singular or joint-limit configuration: joint speeds spike or the "
        "end-effector stops following the commanded direction."
    ),
    "STALLED": "Commands are being sent but the end-effector is not moving and no progress is made.",
    "GOAL_REACHED": "The task goal for the current phase has been achieved.",
}

GUARD_QUESTIONS: Dict[str, Question] = {
    "is_anomaly_detected": NoulQuestion(
        instructions=(
            "Is the end-effector experiencing an unmodeled collision or stall "
            "(contact force or torque it was not commanded to produce, or no motion "
            "despite a motion command)?"
        ),
    ),
    "policy_execution_state": ChoiceQuestion(
        instructions="What is the current state of the robot policy execution?",
        options=POLICY_STATES,
    ),
    "grasp_stability": ScoreQuestion(
        instructions="Rate the physical stability of the current grasp.",
        levels={
            1: "Object dropped or not in the gripper at all.",
            2: "Object held but visibly slipping or rotating in the fingers.",
            3: "Object held with some compliance; small relative motion is possible.",
            4: "Object held firmly with only minor force fluctuations.",
            5: "Object rigidly held; gripper force steady and object pose fixed relative to the gripper.",
        },
    ),
    "intervention_urgency": ScoreQuestion(
        instructions="How urgently does a human or safety system need to intervene right now?",
        levels={
            1: "No intervention needed; autonomous execution is healthy.",
            2: "Worth logging for later review, but no action now.",
            3: "A human should watch closely; a retry or replan may be needed soon.",
            4: "Pause autonomy and hand control to a teleoperator.",
            5: "Immediate stop: risk of damage to the robot, the object, or people.",
        },
    ),
}
