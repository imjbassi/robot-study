from pathlib import Path

import pytest

from jev_guard.decision import Action, DecisionEngine, GuardConfig
from jev_guard.jev_client import MockJevClient
from jev_guard.replay import load_episodes, replay_episode
from jev_guard.schemas import ChoiceAnswer, NoulAnswer, ScoreAnswer
from jev_guard.telemetry import Sample, TelemetryWindow, render_state

EPISODES = load_episodes(str(Path(__file__).with_name("mock_robot_states.json")))


def _sample(t, **kw):
    base = dict(
        t=t, phase="TRANSPORT", cmd_ee_speed=0.1, ee_speed=0.1, ee_force=5.0,
        gripper_width=0.04, gripper_force=20.0, object_visible=True, object_offset=0.012,
        goal_distance=0.3, joint_speed_max=0.5, joint_limit_margin=0.5,
    )
    base.update(kw)
    return Sample(**base)


def _features(**kw):
    w = TelemetryWindow()
    for i in range(10):
        w.add(_sample(i * 0.05, **kw))
    return w.features()


def _answers(p=0.05, state="ON_TRACK", grasp=4.5, urgency=1.2):
    return {
        "is_anomaly_detected": NoulAnswer(p),
        "policy_execution_state": ChoiceAnswer(state, {state: 0.9}, 0.9),
        "grasp_stability": ScoreAnswer(grasp),
        "intervention_urgency": ScoreAnswer(urgency),
    }


# --------------------------------------------------------------------------- #
# Telemetry
# --------------------------------------------------------------------------- #
def test_window_trims_by_age():
    w = TelemetryWindow(window_s=1.0)
    for i in range(100):
        w.add(_sample(i * 0.05))
    f = w.features()
    assert f.window_s <= 1.0 + 1e-9


def test_stall_duration():
    w = TelemetryWindow()
    for i in range(20):
        w.add(_sample(i * 0.05, ee_speed=0.1 if i < 10 else 0.0))
    assert w.features().stall_duration_s == pytest.approx(0.5, abs=1e-6)


def test_render_mentions_key_facts():
    w = TelemetryWindow()
    for i in range(20):
        w.add(_sample(i * 0.05, ee_speed=0.1 if i < 10 else 0.0,
                      ee_force=5.0 if i < 17 else 30.0))
    text = render_state(w.features())
    assert "TRANSPORT" in text
    assert "has not moved" in text
    assert "sudden spike" in text


# --------------------------------------------------------------------------- #
# Decision engine
# --------------------------------------------------------------------------- #
def test_healthy_continues():
    e = DecisionEngine()
    assert e.decide(_features(), _answers()).action == Action.CONTINUE


def test_estop_requires_both_probability_and_urgency():
    e = DecisionEngine()
    assert e.decide(_features(), _answers(p=0.95, urgency=3.0)).action == Action.CONTINUE
    e = DecisionEngine()
    assert e.decide(_features(), _answers(p=0.95, urgency=4.5)).action == Action.ESTOP


def test_estop_latches_until_reset():
    e = DecisionEngine()
    e.decide(_features(), _answers(p=0.95, urgency=4.5))
    assert e.decide(_features(), _answers()).action == Action.ESTOP
    e.reset()
    assert e.decide(_features(), _answers()).action == Action.CONTINUE


def test_handover_is_debounced():
    e = DecisionEngine(GuardConfig(handover_confirm_ticks=2))
    f = _features()
    assert e.decide(f, _answers(state="OBJECT_SLIPPED")).action == Action.CONTINUE
    assert e.decide(f, _answers(state="OBJECT_SLIPPED")).action == Action.TELEOP_HANDOVER
    # a healthy answer resets the streak
    e2 = DecisionEngine(GuardConfig(handover_confirm_ticks=2))
    e2.decide(f, _answers(state="OBJECT_SLIPPED"))
    e2.decide(f, _answers())
    assert e2.decide(f, _answers(state="OBJECT_SLIPPED")).action == Action.CONTINUE


def test_low_grasp_only_matters_while_holding():
    cfg = GuardConfig(handover_confirm_ticks=1)
    assert DecisionEngine(cfg).decide(_features(), _answers(grasp=1.2)).action == Action.TELEOP_HANDOVER
    approach = _features(phase="APPROACH")
    assert DecisionEngine(cfg).decide(approach, _answers(grasp=1.2)).action == Action.CONTINUE


def test_local_force_limit_wins_without_jev():
    e = DecisionEngine()
    v = e.decide(_features(ee_force=80.0), None)
    assert v.action == Action.ESTOP
    assert v.source == "local"


def test_stale_telemetry_hands_over():
    w = TelemetryWindow()
    w.add(_sample(0.0))
    f = w.features(now=1.0)
    assert DecisionEngine().decide(f, _answers()).action == Action.TELEOP_HANDOVER


def test_jev_loss_degrades_to_handover():
    e = DecisionEngine(GuardConfig(max_consecutive_jev_failures=2))
    f = _features()
    e.record_jev_failure()
    assert e.decide(f, None).action == Action.CONTINUE
    e.record_jev_failure()
    v = e.decide(f, None)
    assert v.action == Action.TELEOP_HANDOVER and v.source == "degraded"
    assert e.decide(f, _answers()).action == Action.CONTINUE  # recovers on fresh answers


# --------------------------------------------------------------------------- #
# End-to-end replay with the offline mock backend
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("episode", EPISODES, ids=lambda e: e["name"])
def test_replay_matches_expected_action(episode):
    r = replay_episode(episode, MockJevClient())
    assert r.worst.name == episode["expected_action"]
    if episode["event_t"] is not None:
        # no false alarms before the injected fault, and detection within 1 s
        assert r.first_trigger_t >= episode["event_t"]
        assert r.detection_delay_s <= 1.0
