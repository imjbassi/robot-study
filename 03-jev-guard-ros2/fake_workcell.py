"""A fake pick-and-place workcell that feeds jev_guard every topic it listens to.

    python3 fake_workcell.py                      # clean runs, forever
    python3 fake_workcell.py --fault collision    # inject a fault on every run
    python3 fake_workcell.py --fault stall|slip|singularity

Every run goes APPROACH -> GRASP -> LIFT -> TRANSPORT -> PLACE -> RETREAT and then
starts over. The fault (if any) is injected partway through the phase it
belongs to. Everything is published at 50 Hz:

    /joint_states         sensor_msgs/JointState        Panda joints + gripper
    /ee_pose              geometry_msgs/PoseStamped     end-effector position
    /ee_twist             geometry_msgs/TwistStamped    measured EE velocity
    /cmd_ee_twist         geometry_msgs/TwistStamped    what the "policy" commands
    /ee_wrench            geometry_msgs/WrenchStamped   external force on the EE
    /goal_pose            geometry_msgs/PoseStamped     where the object should go
    /camera/detections    vision_msgs/Detection3DArray  the target object
    /task_phase           std_msgs/String               current phase

It also acts as the "hardware safety bridge": while /jev_guard/estop is true the
arm freezes, and it resumes after `ros2 service call /jev_guard/reset std_srvs/srv/Trigger`.
It is fail-safe: the guard publishes /jev_guard/estop every tick, and if that
heartbeat is missing for HEARTBEAT_TIMEOUT_S (guard not started, crashed, network
down) the arm freezes too. Silence must never mean "keep going".
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

RATE_HZ = 50.0
SPEED = 0.10  # m/s, commanded EE speed while moving
HEARTBEAT_TIMEOUT_S = 0.5  # guard ticks at 10 Hz, so this is ~5 missed ticks

# Must match config/guard.yaml
ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
UPPER = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
HOME = [0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.8]
GRIPPER = "panda_finger_joint1"

START = (0.30, 0.00, 0.40)
OBJECT = (0.50, 0.00, 0.05)
PLACE = (0.50, 0.30, 0.05)
LIFT_Z = 0.25

# Which phase each fault happens in, and how far into that phase (seconds).
FAULTS = {"collision": ("APPROACH", 1.0), "stall": ("APPROACH", 1.0),
          "slip": ("TRANSPORT", 1.0), "singularity": ("TRANSPORT", 1.0)}


def lerp(a, b, s):
    return tuple(x + (y - x) * s for x, y in zip(a, b))


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class FakeWorkcell(Node):
    def __init__(self, fault):
        super().__init__("fake_workcell")
        self.fault = fault
        pub = self.create_publisher
        sensor = qos_profile_sensor_data
        self.joints_pub = pub(JointState, "/joint_states", sensor)
        self.pose_pub = pub(PoseStamped, "/ee_pose", sensor)
        self.twist_pub = pub(TwistStamped, "/ee_twist", sensor)
        self.cmd_pub = pub(TwistStamped, "/cmd_ee_twist", sensor)
        self.wrench_pub = pub(WrenchStamped, "/ee_wrench", sensor)
        self.goal_pub = pub(PoseStamped, "/goal_pose", 10)
        self.dets_pub = pub(Detection3DArray, "/camera/detections", sensor)
        self.phase_pub = pub(String, "/task_phase", 10)
        self.create_subscription(Bool, "/jev_guard/estop", self.on_estop, 10)

        self.estopped = False
        self.last_heartbeat = None
        self.heartbeat_ok = False
        self.run = 0
        self.start_run()
        self.create_timer(1.0 / RATE_HZ, self.step)
        self.get_logger().info(f"workcell up at {RATE_HZ:.0f} Hz, fault={fault or 'none'}; waiting for guard heartbeat")

    # ------------------------------------------------------------------ #
    def start_run(self):
        self.run += 1
        self.ee = START
        self.obj = OBJECT
        self.gripper_width = 0.08
        self.grip_force = 0.0
        self.phase = None
        self.set_phase("APPROACH", START, OBJECT)

    def set_phase(self, phase, frm, to):
        """Move the EE from `frm` to `to` in a straight line at SPEED."""
        self.phase = phase
        self.frm, self.to = frm, to
        self.phase_t = 0.0
        self.fault_elapsed = 0.0
        self.duration = max(dist(frm, to) / SPEED, 0.5)
        self.get_logger().info(f"run {self.run}: {phase}")

    def on_estop(self, msg):
        self.last_heartbeat = self.get_clock().now()
        if msg.data != self.estopped:
            self.estopped = msg.data
            self.get_logger().warn("E-STOP: arm frozen" if msg.data else "E-stop cleared: resuming")

    # ------------------------------------------------------------------ #
    def guard_alive(self):
        alive = self.last_heartbeat is not None and (
            (self.get_clock().now() - self.last_heartbeat).nanoseconds * 1e-9 < HEARTBEAT_TIMEOUT_S)
        if alive != self.heartbeat_ok:
            self.heartbeat_ok = alive
            if alive:
                self.get_logger().info("guard heartbeat OK: moving")
            else:
                self.get_logger().error(f"no guard heartbeat for {HEARTBEAT_TIMEOUT_S} s: arm frozen")
        return alive

    def step(self):
        dt = 1.0 / RATE_HZ
        cmd_v = measured_v = 0.0
        force = 3.0 + 0.5 * math.sin(self.phase_t * 7.0)  # gravity/sensor noise
        joint_speed, near_limit = 0.3, False

        fault_phase, fault_at = FAULTS.get(self.fault, (None, 0.0))
        in_fault = self.phase == fault_phase and self.phase_t >= fault_at

        if self.guard_alive() and not self.estopped:
            if in_fault:
                self.fault_elapsed += dt
            else:
                self.phase_t += dt
            s = min(self.phase_t / self.duration, 1.0)
            cmd_v = measured_v = SPEED if s < 1.0 else 0.0

            if in_fault and self.fault in ("collision", "stall"):
                # The policy keeps commanding motion but the arm doesn't move.
                measured_v = 0.0
                if self.fault == "collision":
                    force = 35.0  # hit something: big unexpected contact force
            else:
                self.ee = lerp(self.frm, self.to, s)

            if self.phase in ("LIFT", "TRANSPORT", "PLACE"):
                if in_fault and self.fault == "slip":
                    # Object slides out of the fingers and grip force drains away.
                    self.obj = (self.obj[0], self.obj[1] - 0.05 * dt, self.obj[2] - 0.05 * dt)
                    self.grip_force = max(0.0, self.grip_force - 20.0 * dt)
                else:
                    self.obj = (self.ee[0], self.ee[1], self.ee[2] - 0.01)

            if in_fault and self.fault == "singularity":
                # Wrist folds up against its joint limit and joint speeds spike.
                near_limit, joint_speed = True, 2.5

            if self.phase == "GRASP":
                self.gripper_width = 0.08 - 0.05 * s
                self.grip_force = 25.0 * s

            if self.fault_elapsed > 3.0:
                self.get_logger().info(f"run {self.run}: {self.fault} fault went on for 3 s, restarting")
                self.start_run()
            elif s >= 1.0:
                self.next_phase()

        self.publish(cmd_v, measured_v, force, joint_speed, near_limit)

    def next_phase(self):
        above_obj = (OBJECT[0], OBJECT[1], LIFT_Z)
        above_place = (PLACE[0], PLACE[1], LIFT_Z)
        if self.phase == "APPROACH":
            self.set_phase("GRASP", self.ee, self.ee)
        elif self.phase == "GRASP":
            self.set_phase("LIFT", self.ee, above_obj)
        elif self.phase == "LIFT":
            self.set_phase("TRANSPORT", self.ee, above_place)
        elif self.phase == "TRANSPORT":
            self.set_phase("PLACE", self.ee, (PLACE[0], PLACE[1], PLACE[2] + 0.01))
        elif self.phase == "PLACE":
            self.gripper_width, self.grip_force = 0.08, 0.0
            self.set_phase("RETREAT", self.ee, START)
        else:
            self.start_run()

    # ------------------------------------------------------------------ #
    def publish(self, cmd_v, measured_v, force, joint_speed, near_limit):
        stamp = self.get_clock().now().to_msg()
        direction = [b - a for a, b in zip(self.frm, self.to)]
        norm = math.sqrt(sum(d * d for d in direction)) or 1.0
        unit = [d / norm for d in direction]

        js = JointState()
        js.header.stamp = stamp
        js.name = ARM_JOINTS + [GRIPPER]
        js.position = [q + 0.05 * math.sin(self.phase_t + i) for i, q in enumerate(HOME)]
        if near_limit:
            # 0.06 rad from joint 4's upper limit: inside the guard's "near a limit"
            # band (0.10) but outside its local E-stop margin (0.03).
            js.position[3] = UPPER[3] - 0.06
        js.position.append(self.gripper_width / 2.0)  # guard multiplies by gripper_width_scale=2
        js.velocity = [joint_speed if cmd_v else 0.0] * len(ARM_JOINTS) + [0.0]
        js.effort = [0.0] * len(ARM_JOINTS) + [self.grip_force]
        self.joints_pub.publish(js)

        def pose(p):
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = stamp, "base"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p
            m.pose.orientation.w = 1.0
            return m

        def twist(v):
            m = TwistStamped()
            m.header.stamp, m.header.frame_id = stamp, "base"
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (u * v for u in unit)
            return m

        self.pose_pub.publish(pose(self.ee))
        self.goal_pub.publish(pose(PLACE))
        self.cmd_pub.publish(twist(cmd_v))
        self.twist_pub.publish(twist(measured_v))

        w = WrenchStamped()
        w.header.stamp, w.header.frame_id = stamp, "ee"
        w.wrench.force.z = force
        self.wrench_pub.publish(w)

        dets = Detection3DArray()
        dets.header.stamp, dets.header.frame_id = stamp, "base"
        d = Detection3D()
        d.header = dets.header
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "target"
        hyp.hypothesis.score = 0.95
        d.results.append(hyp)
        d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z = self.obj
        dets.detections.append(d)
        self.dets_pub.publish(dets)

        self.phase_pub.publish(String(data=self.phase))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fault", choices=sorted(FAULTS), default=None)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = FakeWorkcell(args.fault)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
