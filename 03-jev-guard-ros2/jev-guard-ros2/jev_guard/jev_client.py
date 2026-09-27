"""Minimal, dependency-free client for Jev's System One endpoint.

All guard questions are sent in **one** batched request so the model evaluates
them together against the same state, and the round trip is paid once.

Only the standard library is used so the package installs cleanly inside a
ROS 2 workspace without extra pip dependencies.

The wire format follows TypeSafe's public examples::

    POST {base_url}/v1/systemone
    {"state": "...", "model": "jev-latest", "questions": {name: {...}}}
    -> {"model": "...", "answers": {name: {...}}, "usage": {...}}

Answer parsing is deliberately tolerant of field-name variations so a small
API revision does not take the safety monitor down; anything it cannot parse
raises ``JevResponseError`` and the guard falls back to its local rules.
"""

from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional

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

DEFAULT_BASE_URL = "https://api.typesafe.ai"
DEFAULT_MODEL = "jev-latest"
API_KEY_ENV = "TYPESAFE_API_KEY"


class JevError(RuntimeError):
    """Base error for the Jev client."""


class JevTimeoutError(JevError):
    """The request did not complete within the configured deadline."""


class JevResponseError(JevError):
    """The response could not be parsed into typed answers."""


@dataclass
class JevResult:
    answers: Dict[str, Answer]
    latency_ms: float
    model: Optional[str] = None
    usage: Dict[str, int] = field(default_factory=dict)


# --------------------------------------------------------------------------- #
# Request / response translation
# --------------------------------------------------------------------------- #
def build_request(
    state: str, questions: Mapping[str, Question], model: str = DEFAULT_MODEL
) -> Dict[str, Any]:
    if not questions:
        raise ValueError("at least one question is required")
    return {
        "state": state,
        "model": model,
        "questions": {name: q.to_wire() for name, q in questions.items()},
    }


def _first(d: Mapping[str, Any], *keys: str) -> Any:
    for k in keys:
        if k in d and d[k] is not None:
            return d[k]
    return None


def _floats(d: Any) -> Dict[str, float]:
    if not isinstance(d, Mapping):
        return {}
    return {str(k): float(v) for k, v in d.items()}


def parse_answer(raw: Mapping[str, Any], question: Question) -> Answer:
    if isinstance(question, NoulQuestion):
        p = _first(raw, "noul", "probability", "p")
        if p is None:
            raise JevResponseError(f"noul answer missing probability: {raw!r}")
        p = float(p)
        if not 0.0 <= p <= 1.0:
            raise JevResponseError(f"noul probability out of range: {p}")
        return NoulAnswer(probability=p)

    if isinstance(question, ChoiceQuestion):
        probs = _floats(_first(raw, "probabilities", "distribution", "scores"))
        label = _first(raw, "choice", "label", "answer")
        if label is None and probs:
            label = max(probs, key=probs.get)
        if label is None:
            raise JevResponseError(f"choice answer missing label: {raw!r}")
        label = str(label)
        if label not in question.options:
            raise JevResponseError(f"choice label {label!r} not in {question.labels}")
        conf = _first(raw, "confidence")
        return ChoiceAnswer(
            label=label,
            probabilities=probs,
            confidence=float(conf) if conf is not None else None,
        )

    if isinstance(question, ScoreQuestion):
        level_probs = _floats(_first(raw, "probabilities", "level_probabilities", "distribution"))
        score = _first(raw, "score", "expected_score", "value")
        if score is None and level_probs:
            score = sum(float(k) * v for k, v in level_probs.items()) / (
                sum(level_probs.values()) or 1.0
            )
        if score is None:
            raise JevResponseError(f"score answer missing score: {raw!r}")
        score = float(score)
        if not question.min_level - 1e-6 <= score <= question.max_level + 1e-6:
            raise JevResponseError(
                f"score {score} outside [{question.min_level}, {question.max_level}]"
            )
        conf = _first(raw, "confidence")
        return ScoreAnswer(
            score=score,
            level_probabilities=level_probs,
            confidence=float(conf) if conf is not None else None,
        )

    raise TypeError(f"unsupported question type: {type(question).__name__}")


def parse_response(
    body: Mapping[str, Any], questions: Mapping[str, Question]
) -> Dict[str, Answer]:
    raw_answers = body.get("answers")
    if not isinstance(raw_answers, Mapping):
        raise JevResponseError(f"response has no 'answers' object: {body!r}")
    out: Dict[str, Answer] = {}
    for name, q in questions.items():
        if name not in raw_answers:
            raise JevResponseError(f"response missing answer for {name!r}")
        out[name] = parse_answer(raw_answers[name], q)
    return out


# --------------------------------------------------------------------------- #
# Client
# --------------------------------------------------------------------------- #
class JevClient:
    """Synchronous client with an optional background executor.

    ``ask`` blocks; ``submit`` returns a ``Future`` so a ROS timer callback can
    fire a query and pick up the result on a later tick without stalling the
    executor.
    """

    def __init__(
        self,
        api_key: Optional[str] = None,
        base_url: str = DEFAULT_BASE_URL,
        model: str = DEFAULT_MODEL,
        timeout_s: float = 0.25,
        max_in_flight: int = 2,
    ) -> None:
        self.api_key = api_key or os.environ.get(API_KEY_ENV)
        if not self.api_key:
            raise JevError(f"no API key: pass api_key or set ${API_KEY_ENV}")
        self.url = base_url.rstrip("/") + "/v1/systemone"
        self.model = model
        self.timeout_s = timeout_s
        self._pool = ThreadPoolExecutor(max_workers=max_in_flight, thread_name_prefix="jev")

    def ask(self, state: str, questions: Mapping[str, Question], features=None) -> JevResult:
        # ``features`` is accepted for interface parity with MockJevClient; the
        # real model only sees the text ``state``.
        payload = json.dumps(build_request(state, questions, self.model)).encode()
        req = urllib.request.Request(
            self.url,
            data=payload,
            method="POST",
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
        )
        t0 = time.perf_counter()
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                body = json.loads(resp.read())
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors="replace")[:500]
            raise JevError(f"HTTP {e.code} from Jev: {detail}") from e
        except (TimeoutError, urllib.error.URLError) as e:
            if isinstance(e, TimeoutError) or isinstance(getattr(e, "reason", None), TimeoutError):
                raise JevTimeoutError(f"Jev request exceeded {self.timeout_s:.3f}s") from e
            raise JevError(f"Jev request failed: {e}") from e
        latency_ms = (time.perf_counter() - t0) * 1000.0

        return JevResult(
            answers=parse_response(body, questions),
            latency_ms=latency_ms,
            model=body.get("model"),
            usage={k: int(v) for k, v in (body.get("usage") or {}).items()},
        )

    def submit(
        self, state: str, questions: Mapping[str, Question], features=None
    ) -> "Future[JevResult]":
        return self._pool.submit(self.ask, state, questions, features)

    def close(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)


class MockJevClient:
    """Offline stand-in used for tests, CI, and running the node without a key.

    It is **not** a model: it applies simple hand-written rules to the
    structured telemetry summary so the rest of the pipeline can be exercised
    end to end. ``latency_ms`` simulates network + inference time.
    """

    def __init__(self, latency_ms: float = 0.0) -> None:
        self.latency_ms = latency_ms
        self._pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="jev-mock")

    def ask(self, state: str, questions: Mapping[str, Question], features=None) -> JevResult:
        from .telemetry import heuristic_answers  # local import avoids a cycle

        t0 = time.perf_counter()
        if self.latency_ms:
            time.sleep(self.latency_ms / 1000.0)
        answers = heuristic_answers(features, questions) if features is not None else {}
        missing = set(questions) - set(answers)
        if missing:
            raise JevResponseError(f"mock cannot answer {sorted(missing)} without features")
        return JevResult(
            answers=answers,
            latency_ms=(time.perf_counter() - t0) * 1000.0,
            model="mock",
        )

    def submit(self, state, questions, features=None):
        return self._pool.submit(self.ask, state, questions, features)

    def close(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)
