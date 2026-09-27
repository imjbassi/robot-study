"""ROS 2 node: real-time safety and milestone watchdog for a VLA policy rollout.

Data flow per tick (default 10 Hz)::

    topics --> latest-value cache --(50 Hz sampler)--> TelemetryWindow
    TelemetryWindow --features--> render_state --> Jev (async, batched)
    features + newest Jev answers --> DecisionEngine --> /jev_guard/estop
                                                     --> /jev_guard/teleop_request
                                                     --> /jev_guard/status (JSON)

The Jev query never blocks the executor: each tick collects the previous
query's result (if it has arrived) and submits a new one. Local hard limits
are evaluated every tick regardless of the network.

Positions from ``/ee_pose``, ``/goal_pose`` and ``/camera/detections`` are
assumed to be expressed in the same frame (typically the robot base frame).
"""

from __future__ import annotations

import json
import math
from concurrent.futures import Future
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection3DArray

from .decision import Action, DecisionEngine, GuardConfig
from .jev_client import JevClient, JevError, JevResult, MockJevClient
from .schemas import GUARD_QUESTIONS
from .telemetry import HOLDING_PHASES, Sample, TelemetryWindow, render_state

Vec3 = Tuple[float, float, float]


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def _dist(a: Vec3, b: Vec3) -> float:
    return _norm((a[0] - b[0], a[1] - b[1], a[2] - b[2]))


def _xyz(p) -> Vec3:
    return (p.x, p.y, p.z)


class JevGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("jev_guard")

        p = self.declare_parameter
        self.tick_hz = p("tick_hz", 10.0).value
        self.sample_hz = p("sample_hz", 50.0).value
        self.window_s = p("window_s", 1.5).value
        self.max_result_age_s = p("max_result_age_s", 0.3).value
        use_mock = p("use_mock", False).value
        mock_latency_ms = p("mock_latency_ms", 80.0).value
        base_url = p("jev.base_url", "https://api.typesafe.ai").value
        model = p("jev.model", "jev-latest").value
        timeout_s = p("jev.timeout_s", 0.25).value

        self.arm_joints: List[str] = list(p("arm_joint_names", [""]).value)
        self.lower: List[float] = list(p("joint_lower_limits", [0.0]).value)
        self.upper: List[float] = list(p("joint_upper_limits", [0.0]).value)
        self.gripper_joint = p("gripper_joint_name", "").value
        self.gripper_width_scale = p("gripper_width_scale", 2.0).value
        self.object_class = p("object_class_id", "target").value

        cfg = GuardConfig(
            anomaly_estop_prob=p("anomaly_estop_prob", 0.85).value,
            urgency_estop=p("urgency_estop", 4.0).value,
            urgency_handover=p("urgency_handover", 4.0).value,
            grasp_handover_below=p("grasp_handover_below", 2.0).value,
            handover_confirm_ticks=p("handover_confirm_ticks", 2).value,
            max_ee_force_n=p("max_ee_force_n", 60.0).value,
            min_joint_limit_margin_rad=p("min_joint_limit_margin_rad", 0.03).value,
            max_telemetry_age_s=p("max_telemetry_age_s", 0.25).value,
            max_consecutive_jev_failures=p("max_consecutive_jev_failures", 3).value,
            handover_on_jev_loss=p("handover_on_jev_loss", True).value,
        )
        self.engine = DecisionEngine(cfg)
        self.window = TelemetryWindow(self.window_s)

        if use_mock:
            self.client = MockJevClient(latency_ms=mock_latency_ms)
            self.get_logger().warn("use_mock=true: Jev answers come from local heuristics")
        else:
            self.client = JevClient(base_url=base_url, model=model, timeout_s=timeout_s)

        # Latest-value cache
        self._phase = "APPROACH"
        self._cmd_speed = 0.0
        self._ee_speed = 0.0
        self._ee_force = 0.0
        self._ee_pos: Optional[Vec3] = None
        self._goal_pos: Optional[Vec3] = None
        self._obj_pos: Optional[Vec3] = None
        self._obj_stamp = 0.0
        self._gripper_width = 0.0
        self._gripper_force = 0.0
        self._joint_speed_max = 0.0
        self._joint_margin = math.inf
        self._last_msg_t: Optional[float] = None

        self._inflight: Optional[Tuple[float, Future]] = None
        self._latencies: List[float] = []
        self._last_logged: Tuple[Action, str] = (Action.CONTINUE, "jev")

        sensor = qos_profile_sensor_data
        self.create_subscription(JointState, "/joint_states", self._on_joints, sensor)
        self.create_subscription(TwistStamped, "/ee_twist", self._on_ee_twist, sensor)
        self.create_subscription(TwistStamped, "/cmd_ee_twist", self._on_cmd_twist, sensor)
        self.create_subscription(WrenchStamped, "/ee_wrench", self._on_wrench, sensor)
        self.create_subscription(PoseStamped, "/ee_pose", self._on_ee_pose, sensor)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.create_subscription(Detection3DArray, "/camera/detections", self._on_dets, sensor)
        self.create_subscription(String, "/task_phase", self._on_phase, 10)

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.estop_pub = self.create_publisher(Bool, "/jev_guard/estop", reliable)
        self.teleop_pub = self.create_publisher(Bool, "/jev_guard/teleop_request", reliable)
        self.status_pub = self.create_publisher(String, "/jev_guard/status", 10)
        self.create_service(Trigger, "/jev_guard/reset", self._on_reset)

        self.create_timer(1.0 / self.sample_hz, self._sample)
        self.create_timer(1.0 / self.tick_hz, self._tick)
        self.get_logger().info(
            f"jev_guard up: tick {self.tick_hz:.0f} Hz, window {self.window_s:.1f} s, "
            f"backend {'mock' if use_mock else base_url}"
        )

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _touch(self) -> None:
        self._last_msg_t = self._now()

    def _on_joints(self, msg: JointState) -> None:
        idx: Dict[str, int] = {n: i for i, n in enumerate(msg.name)}
        speeds, margins = [], []
        for j, name in enumerate(self.arm_joints):
            i = idx.get(name)
            if i is None:
                continue
            if i < len(msg.velocity):
                speeds.append(abs(msg.velocity[i]))
            if i < len(msg.position) and j < len(self.lower) and j < len(self.upper):
                q = msg.position[i]
                margins.append(min(q - self.lower[j], self.upper[j] - q))
        if speeds:
            self._joint_speed_max = max(speeds)
        if margins:
            self._joint_margin = min(margins)
        g = idx.get(self.gripper_joint)
        if g is not None:
            if g < len(msg.position):
                self._gripper_width = msg.position[g] * self.gripper_width_scale
            if g < len(msg.effort):
                self._gripper_force = abs(msg.effort[g])
        self._touch()

    def _on_ee_twist(self, msg: TwistStamped) -> None:
        self._ee_speed = _norm(_xyz(msg.twist.linear))
        self._touch()

    def _on_cmd_twist(self, msg: TwistStamped) -> None:
        self._cmd_speed = _norm(_xyz(msg.twist.linear))

    def _on_wrench(self, msg: WrenchStamped) -> None:
        self._ee_force = _norm(_xyz(msg.wrench.force))
        self._touch()

    def _on_ee_pose(self, msg: PoseStamped) -> None:
        self._ee_pos = _xyz(msg.pose.position)

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal_pos = _xyz(msg.pose.position)

    def _on_dets(self, msg: Detection3DArray) -> None:
        best, best_score = None, -1.0
        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id == self.object_class and hyp.hypothesis.score > best_score:
                    best, best_score = det, hyp.hypothesis.score
        if best is not None:
            self._obj_pos = _xyz(best.bbox.center.position)
            self._obj_stamp = self._now()

    def _on_phase(self, msg: String) -> None:
        phase = msg.data.strip().upper()
        if phase != self._phase:
            self.get_logger().info(f"phase {self._phase} -> {phase}")
        self._phase = phase

    def _on_reset(self, _req, resp):
        was = self.engine.latched
        self.engine.reset()
        resp.success = True
        resp.message = "E-stop latch cleared" if was else "not latched"
        self.get_logger().warn(f"reset requested: {resp.message}")
        return resp

    # ------------------------------------------------------------------ #
    # Sampling and guard tick
    # ------------------------------------------------------------------ #
    def _sample(self) -> None:
        if self._last_msg_t is None:
            return
        t = self._last_msg_t
        visible = self._obj_pos is not None and (self._now() - self._obj_stamp) < 0.5
        offset = goal_d = None
        if visible and self._ee_pos is not None:
            offset = _dist(self._obj_pos, self._ee_pos)
        if self._phase in HOLDING_PHASES:
            if visible and self._goal_pos is not None:
                goal_d = _dist(self._obj_pos, self._goal_pos)
        elif self._phase in ("APPROACH", "GRASP"):
            goal_d = offset
        self.window.add(
            Sample(
                t=t,
                phase=self._phase,
                cmd_ee_speed=self._cmd_speed,
                ee_speed=self._ee_speed,
                ee_force=self._ee_force,
                gripper_width=self._gripper_width,
                gripper_force=self._gripper_force,
                object_visible=visible,
                object_offset=offset,
                goal_distance=goal_d,
                joint_speed_max=self._joint_speed_max,
                joint_limit_margin=self._joint_margin if math.isfinite(self._joint_margin) else 1.0,
            )
        )

    def _collect(self, now: float) -> Optional[JevResult]:
        if self._inflight is None:
            return None
        sent_at, fut = self._inflight
        if not fut.done():
            if now - sent_at > self.max_result_age_s:
                fut.cancel()
                self._inflight = None
                self.engine.record_jev_failure()
                self.get_logger().warn("Jev result overdue; dropped")
            return None
        self._inflight = None
        try:
            res = fut.result()
        except JevError as e:
            self.engine.record_jev_failure()
            self.get_logger().warn(f"Jev query failed: {e}")
            return None
        if now - sent_at > self.max_result_age_s:
            self.engine.record_jev_failure()
            return None
        self._latencies = (self._latencies + [res.latency_ms])[-200:]
        return res

    def _tick(self) -> None:
        now = self._now()
        feats = self.window.features(now=now)

        result = self._collect(now)
        if self._inflight is None and feats is not None and not self.engine.latched:
            self._inflight = (
                now,
                self.client.submit(render_state(feats), GUARD_QUESTIONS, features=feats),
            )

        verdict = self.engine.decide(feats, result.answers if result else None)

        self.estop_pub.publish(Bool(data=verdict.action == Action.ESTOP))
        self.teleop_pub.publish(Bool(data=verdict.action == Action.TELEOP_HANDOVER))

        status = verdict.as_dict()
        status["phase"] = self._phase
        status["latched"] = self.engine.latched
        if result is not None:
            status["jev_latency_ms"] = round(result.latency_ms, 1)
        if self._latencies:
            s = sorted(self._latencies)
            status["jev_latency_p50_ms"] = round(s[len(s) // 2], 1)
            status["jev_latency_p95_ms"] = round(s[min(len(s) - 1, int(len(s) * 0.95))], 1)
        self.status_pub.publish(String(data=json.dumps(status)))

        # Log transitions only; the status topic carries the per-tick detail.
        key = (verdict.action, verdict.source)
        if key != self._last_logged:
            self._last_logged = key
            if verdict.action == Action.CONTINUE:
                self.get_logger().info(f"CONTINUE [{verdict.source}]")
            else:
                # rclpy ties each logging call site to one severity, so error and
                # warn need separate calls (a shared call raises ValueError).
                msg = f"{verdict.action.name} [{verdict.source}]: " + "; ".join(verdict.reasons)
                if verdict.action == Action.ESTOP:
                    self.get_logger().error(msg)
                else:
                    self.get_logger().warn(msg)

    def destroy_node(self) -> None:
        self.client.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JevGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
