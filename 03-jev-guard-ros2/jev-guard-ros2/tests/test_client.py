import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from jev_guard.jev_client import (
    JevClient,
    JevError,
    JevResponseError,
    JevTimeoutError,
    build_request,
    parse_response,
)
from jev_guard.schemas import (
    GUARD_QUESTIONS,
    ChoiceAnswer,
    ChoiceQuestion,
    NoulAnswer,
    ScoreAnswer,
    ScoreQuestion,
)

GOOD_ANSWERS = {
    "is_anomaly_detected": {"type": "noul", "noul": 0.91},
    "policy_execution_state": {
        "type": "choice",
        "choice": "STALLED",
        "probabilities": {"ON_TRACK": 0.1, "STALLED": 0.85, "OBJECT_SLIPPED": 0.05},
        "confidence": 0.8,
    },
    "grasp_stability": {"type": "score", "score": 2.3, "confidence": 0.6},
    "intervention_urgency": {
        "type": "score",
        "probabilities": {"1": 0.0, "2": 0.0, "3": 0.2, "4": 0.5, "5": 0.3},
    },
}


# --------------------------------------------------------------------------- #
# Schemas and wire format
# --------------------------------------------------------------------------- #
def test_build_request_batches_all_questions():
    req = build_request("state text", GUARD_QUESTIONS)
    assert req["state"] == "state text"
    assert req["model"] == "jev-latest"
    assert set(req["questions"]) == set(GUARD_QUESTIONS)
    assert req["questions"]["is_anomaly_detected"]["type"] == "noul"
    assert req["questions"]["policy_execution_state"]["type"] == "choice"
    assert "ON_TRACK" in req["questions"]["policy_execution_state"]["criteria"]
    assert list(req["questions"]["grasp_stability"]["criteria"]) == ["1", "2", "3", "4", "5"]
    json.dumps(req)  # must be serialisable


def test_question_validation():
    with pytest.raises(ValueError):
        ChoiceQuestion("x", {"only": "one"})
    with pytest.raises(ValueError):
        ScoreQuestion("x", {1: "a"})
    with pytest.raises(ValueError):
        ScoreQuestion("x", {i: str(i) for i in range(11)})
    with pytest.raises(ValueError):
        ScoreQuestion("x", {2: "b", 1: "a"})


def test_parse_response_typed_answers():
    ans = parse_response({"answers": GOOD_ANSWERS}, GUARD_QUESTIONS)
    assert isinstance(ans["is_anomaly_detected"], NoulAnswer)
    assert ans["is_anomaly_detected"].probability == pytest.approx(0.91)
    assert isinstance(ans["policy_execution_state"], ChoiceAnswer)
    assert ans["policy_execution_state"].label == "STALLED"
    assert isinstance(ans["grasp_stability"], ScoreAnswer)
    assert ans["grasp_stability"].score == pytest.approx(2.3)
    # score derived from level distribution when not given explicitly
    assert ans["intervention_urgency"].score == pytest.approx(3 * 0.2 + 4 * 0.5 + 5 * 0.3)


def test_parse_rejects_bad_answers():
    bad = dict(GOOD_ANSWERS, is_anomaly_detected={"noul": 1.4})
    with pytest.raises(JevResponseError):
        parse_response({"answers": bad}, GUARD_QUESTIONS)
    bad = dict(GOOD_ANSWERS, policy_execution_state={"choice": "DANCING"})
    with pytest.raises(JevResponseError):
        parse_response({"answers": bad}, GUARD_QUESTIONS)
    bad = {k: v for k, v in GOOD_ANSWERS.items() if k != "grasp_stability"}
    with pytest.raises(JevResponseError):
        parse_response({"answers": bad}, GUARD_QUESTIONS)
    with pytest.raises(JevResponseError):
        parse_response({"error": "nope"}, GUARD_QUESTIONS)


# --------------------------------------------------------------------------- #
# HTTP client against a local fake server
# --------------------------------------------------------------------------- #
class _FakeJev(BaseHTTPRequestHandler):
    delay_s = 0.0
    status = 200
    seen = []

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        type(self).seen.append((self.path, self.headers.get("Authorization"), body))
        time.sleep(type(self).delay_s)
        payload = json.dumps(
            {"model": "jev-test", "answers": GOOD_ANSWERS, "usage": {"input_tokens": 300}}
        ).encode()
        self.send_response(type(self).status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, *a):
        pass


@pytest.fixture
def fake_server():
    _FakeJev.delay_s, _FakeJev.status, _FakeJev.seen = 0.0, 200, []
    srv = HTTPServer(("127.0.0.1", 0), _FakeJev)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()


def test_client_roundtrip(fake_server):
    c = JevClient(api_key="k", base_url=fake_server, timeout_s=2.0)
    res = c.ask("the state", GUARD_QUESTIONS)
    path, auth, body = _FakeJev.seen[0]
    assert path == "/v1/systemone"
    assert auth == "Bearer k"
    assert body["state"] == "the state"
    assert res.model == "jev-test"
    assert res.usage == {"input_tokens": 300}
    assert res.answers["policy_execution_state"].label == "STALLED"
    assert res.latency_ms > 0
    c.close()


def test_client_submit_is_async(fake_server):
    _FakeJev.delay_s = 0.2
    c = JevClient(api_key="k", base_url=fake_server, timeout_s=2.0)
    t0 = time.perf_counter()
    fut = c.submit("s", GUARD_QUESTIONS)
    assert time.perf_counter() - t0 < 0.1
    assert fut.result(timeout=2).answers["is_anomaly_detected"].probability == pytest.approx(0.91)
    c.close()


def test_client_timeout(fake_server):
    _FakeJev.delay_s = 0.5
    c = JevClient(api_key="k", base_url=fake_server, timeout_s=0.1)
    with pytest.raises(JevTimeoutError):
        c.ask("s", GUARD_QUESTIONS)
    c.close()


def test_client_http_error(fake_server):
    _FakeJev.status = 429
    c = JevClient(api_key="k", base_url=fake_server, timeout_s=2.0)
    with pytest.raises(JevError, match="HTTP 429"):
        c.ask("s", GUARD_QUESTIONS)
    c.close()


def test_client_requires_key(monkeypatch):
    monkeypatch.delenv("TYPESAFE_API_KEY", raising=False)
    with pytest.raises(JevError):
        JevClient()
