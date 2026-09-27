"""Latency / cost benchmark: Jev vs. an OpenAI JSON-mode baseline.

Both backends receive the *same* rendered telemetry states from
``mock_robot_states.json`` and must answer the same four guard questions.

Run as a script (recommended) to produce a results file and a Markdown table::

    export TYPESAFE_API_KEY=...        # Jev
    export OPENAI_API_KEY=...          # baseline (optional)
    python3 tests/test_latency.py --n 50 --openai-model gpt-4o

Under pytest the benchmark runs only when the matching API key is set, so CI
stays green without credentials::

    pytest tests/test_latency.py -s

Prices are per million tokens and are CLI flags because they change; check
each provider's current pricing page before quoting results.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Callable, Dict, List, Tuple

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from jev_guard.jev_client import JevClient, parse_answer  # noqa: E402
from jev_guard.replay import load_episodes  # noqa: E402
from jev_guard.schemas import (  # noqa: E402
    GUARD_QUESTIONS,
    ChoiceQuestion,
    NoulQuestion,
    ScoreQuestion,
)
from jev_guard.telemetry import Sample, TelemetryWindow, render_state  # noqa: E402

DATA = Path(__file__).with_name("mock_robot_states.json")


def collect_states(n: int, tick_hz: float = 10.0) -> List[str]:
    """Render guard states at ``tick_hz`` across all episodes, up to ``n``."""
    states: List[str] = []
    for ep in load_episodes(str(DATA)):
        w = TelemetryWindow()
        next_t = 0.0
        for s in ep["samples"]:
            w.add(Sample(**s))
            if s["t"] >= next_t:
                next_t += 1.0 / tick_hz
                states.append(render_state(w.features()))
    step = max(1, len(states) // n)
    return states[::step][:n]


# --------------------------------------------------------------------------- #
# Backends: each returns (latency_ms, input_tokens, output_tokens)
# --------------------------------------------------------------------------- #
def jev_backend(timeout_s: float) -> Callable[[str], Tuple[float, int, int]]:
    client = JevClient(timeout_s=timeout_s)

    def call(state: str):
        r = client.ask(state, GUARD_QUESTIONS)
        return r.latency_ms, r.usage.get("input_tokens", 0), r.usage.get("output_tokens", 0)

    return call


def _openai_prompt() -> str:
    lines = [
        "You are a robot safety monitor. Read the telemetry and answer every question.",
        "Reply with a single JSON object with exactly these keys:",
    ]
    for name, q in GUARD_QUESTIONS.items():
        if isinstance(q, NoulQuestion):
            lines.append(f'- "{name}": {{"noul": <probability 0-1>}}. {q.instructions}')
        elif isinstance(q, ChoiceQuestion):
            opts = "; ".join(f"{k}: {v}" for k, v in q.options.items())
            lines.append(
                f'- "{name}": {{"choice": <one of {list(q.labels)}>, '
                f'"confidence": <0-1>}}. {q.instructions} Options: {opts}'
            )
        elif isinstance(q, ScoreQuestion):
            lv = "; ".join(f"{k}: {v}" for k, v in q.levels.items())
            lines.append(
                f'- "{name}": {{"score": <number {q.min_level}-{q.max_level}>}}. '
                f"{q.instructions} Levels: {lv}"
            )
    return "\n".join(lines)


def openai_backend(model: str, timeout_s: float) -> Callable[[str], Tuple[float, int, int]]:
    from openai import OpenAI

    client = OpenAI(timeout=timeout_s)
    system = _openai_prompt()

    def call(state: str):
        t0 = time.perf_counter()
        resp = client.chat.completions.create(
            model=model,
            temperature=0,
            response_format={"type": "json_object"},
            messages=[{"role": "system", "content": system}, {"role": "user", "content": state}],
        )
        latency = (time.perf_counter() - t0) * 1000.0
        body = json.loads(resp.choices[0].message.content)
        for name, q in GUARD_QUESTIONS.items():  # same validation as the Jev path
            parse_answer(body[name], q)
        return latency, resp.usage.prompt_tokens, resp.usage.completion_tokens

    return call


# --------------------------------------------------------------------------- #
def run(call, states: List[str], warmup: int = 2) -> Dict:
    for s in states[:warmup]:
        try:
            call(s)
        except Exception:
            pass
    lat, tin, tout, errors = [], 0, 0, 0
    for s in states:
        try:
            ms, i, o = call(s)
        except Exception as e:  # count, don't abort: error rate is a result too
            errors += 1
            print(f"  error: {type(e).__name__}: {e}", file=sys.stderr)
            continue
        lat.append(ms)
        tin += i
        tout += o
    lat.sort()
    pct = lambda p: lat[min(len(lat) - 1, int(len(lat) * p))] if lat else float("nan")  # noqa: E731
    return {
        "n": len(states),
        "errors": errors,
        "p50_ms": pct(0.50),
        "p95_ms": pct(0.95),
        "p99_ms": pct(0.99),
        "mean_ms": statistics.fmean(lat) if lat else float("nan"),
        "within_100ms": sum(x <= 100 for x in lat) / len(lat) if lat else 0.0,
        "input_tokens": tin,
        "output_tokens": tout,
    }


def cost_per_1k_calls(r: Dict, in_price: float, out_price: float) -> float:
    ok = r["n"] - r["errors"]
    if not ok:
        return float("nan")
    per_call = (r["input_tokens"] * in_price + r["output_tokens"] * out_price) / 1e6 / ok
    return per_call * 1000


def markdown(results: Dict[str, Dict]) -> str:
    rows = [
        "| Backend | n | errors | p50 (ms) | p95 (ms) | p99 (ms) | <=100 ms | $ / 1k calls |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, r in results.items():
        rows.append(
            f"| {name} | {r['n']} | {r['errors']} | {r['p50_ms']:.0f} | {r['p95_ms']:.0f} | "
            f"{r['p99_ms']:.0f} | {r['within_100ms'] * 100:.0f}% | {r['cost_per_1k']:.4f} |"
        )
    return "\n".join(rows)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--n", type=int, default=50, help="number of states to evaluate")
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--openai-model", default="gpt-4o")
    ap.add_argument("--jev-price-in", type=float, default=0.042, help="$ / 1M input tokens")
    ap.add_argument("--jev-price-out", type=float, default=0.0, help="$ / 1M output tokens")
    ap.add_argument("--openai-price-in", type=float, default=2.50)
    ap.add_argument("--openai-price-out", type=float, default=10.00)
    ap.add_argument("--out", default="benchmark_results.json")
    args = ap.parse_args()

    states = collect_states(args.n)
    results: Dict[str, Dict] = {}

    if os.environ.get("TYPESAFE_API_KEY"):
        print(f"Jev: {len(states)} calls ...")
        r = run(jev_backend(args.timeout), states)
        r["cost_per_1k"] = cost_per_1k_calls(r, args.jev_price_in, args.jev_price_out)
        results["Jev (jev-latest)"] = r
    else:
        print("TYPESAFE_API_KEY not set; skipping Jev")

    if os.environ.get("OPENAI_API_KEY"):
        print(f"OpenAI {args.openai_model}: {len(states)} calls ...")
        r = run(openai_backend(args.openai_model, args.timeout), states)
        r["cost_per_1k"] = cost_per_1k_calls(r, args.openai_price_in, args.openai_price_out)
        results[f"OpenAI {args.openai_model} (JSON mode)"] = r
    else:
        print("OPENAI_API_KEY not set; skipping OpenAI baseline")

    if not results:
        sys.exit("no backends ran; set at least one API key")

    Path(args.out).write_text(json.dumps(results, indent=2))
    print("\n" + markdown(results))
    print(f"\nraw results written to {args.out}")


# --------------------------------------------------------------------------- #
# pytest entry points (skipped without credentials)
# --------------------------------------------------------------------------- #
def test_states_render():
    states = collect_states(20)
    assert len(states) == 20
    assert all("Current phase" in s for s in states)


@pytest.mark.skipif(not os.environ.get("TYPESAFE_API_KEY"), reason="TYPESAFE_API_KEY not set")
def test_jev_latency_budget():
    r = run(jev_backend(timeout_s=2.0), collect_states(20))
    print(r)
    assert r["errors"] == 0
    assert r["p95_ms"] < float(os.environ.get("JEV_P95_BUDGET_MS", "250"))


@pytest.mark.skipif(not os.environ.get("OPENAI_API_KEY"), reason="OPENAI_API_KEY not set")
def test_openai_baseline_runs():
    r = run(openai_backend(os.environ.get("OPENAI_MODEL", "gpt-4o"), 30.0), collect_states(5))
    print(r)
    assert r["errors"] == 0


if __name__ == "__main__":
    main()
