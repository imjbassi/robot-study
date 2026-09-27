"""Offline replay of recorded (or synthetic) telemetry through the guard.

Runs the exact same window -> render -> Jev -> decision path as the ROS node,
without ROS, at a fixed guard rate. Useful for regression tests, threshold
tuning, and benchmarking backends.

    python3 -m jev_guard.replay tests/mock_robot_states.json --mock
    TYPESAFE_API_KEY=... python3 -m jev_guard.replay tests/mock_robot_states.json
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .decision import Action, DecisionEngine, GuardConfig, Verdict
from .jev_client import JevClient, JevError, MockJevClient
from .schemas import GUARD_QUESTIONS
from .telemetry import Sample, TelemetryWindow, render_state


@dataclass
class ReplayResult:
    name: str
    expected: Optional[str]
    event_t: Optional[float]
    verdicts: List[Dict] = field(default_factory=list)
    latencies_ms: List[float] = field(default_factory=list)
    failures: int = 0

    @property
    def worst(self) -> Action:
        return max((Action[v["action"]] for v in self.verdicts), default=Action.CONTINUE)

    @property
    def first_trigger_t(self) -> Optional[float]:
        for v in self.verdicts:
            if v["action"] != "CONTINUE":
                return v["t"]
        return None

    @property
    def detection_delay_s(self) -> Optional[float]:
        if self.event_t is None or self.first_trigger_t is None:
            return None
        return self.first_trigger_t - self.event_t


def load_episodes(path: str) -> List[Dict]:
    with open(path) as f:
        return json.load(f)["episodes"]


def replay_episode(
    episode: Dict,
    client,
    tick_hz: float = 10.0,
    window_s: float = 1.5,
    config: Optional[GuardConfig] = None,
) -> ReplayResult:
    """Replay one episode. Queries are made synchronously on each tick, so the
    decision on tick *k* uses the state at tick *k* (the live node pipelines
    one tick behind; see README)."""
    win = TelemetryWindow(window_s)
    engine = DecisionEngine(config)
    out = ReplayResult(episode["name"], episode.get("expected_action"), episode.get("event_t"))

    period = 1.0 / tick_hz
    next_tick = period
    samples = [Sample(**s) for s in episode["samples"]]
    for i, s in enumerate(samples):
        win.add(s)
        last = i == len(samples) - 1
        if s.t + 1e-9 < next_tick and not last:
            continue
        next_tick += period
        feats = win.features()
        answers = None
        if not engine.latched:
            try:
                res = client.ask(render_state(feats), GUARD_QUESTIONS, features=feats)
                answers = res.answers
                out.latencies_ms.append(res.latency_ms)
            except JevError:
                out.failures += 1
                engine.record_jev_failure()
        v: Verdict = engine.decide(feats, answers)
        d = v.as_dict()
        d["t"] = round(s.t, 3)
        out.verdicts.append(d)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("path")
    ap.add_argument("--mock", action="store_true", help="use local heuristics instead of the API")
    ap.add_argument("--tick-hz", type=float, default=10.0)
    ap.add_argument("--timeout", type=float, default=1.0, help="per-request timeout (s)")
    ap.add_argument("--show-state", action="store_true", help="print the first rendered state")
    args = ap.parse_args()

    client = MockJevClient() if args.mock else JevClient(timeout_s=args.timeout)
    episodes = load_episodes(args.path)

    if args.show_state:
        w = TelemetryWindow()
        for s in episodes[0]["samples"][:30]:
            w.add(Sample(**s))
        print(render_state(w.features()), "\n")

    print(f"{'episode':32} {'expected':16} {'got':16} {'delay':>7} {'p50 ms':>7}")
    ok = 0
    for ep in episodes:
        r = replay_episode(ep, client, tick_hz=args.tick_hz)
        good = r.expected is None or r.worst.name == r.expected
        ok += good
        delay = f"{r.detection_delay_s:.2f}s" if r.detection_delay_s is not None else "-"
        p50 = sorted(r.latencies_ms)[len(r.latencies_ms) // 2] if r.latencies_ms else float("nan")
        print(
            f"{r.name:32} {str(r.expected):16} {r.worst.name:16} {delay:>7} {p50:7.1f}"
            + ("" if good else "   <-- mismatch")
        )
    print(f"\n{ok}/{len(episodes)} episodes matched the expected action")
    client.close()


if __name__ == "__main__":
    main()
