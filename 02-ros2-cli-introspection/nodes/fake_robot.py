"""A fake 6-joint robot arm with a camera, so there's something to inspect.

    python3 fake_robot.py
    python3 fake_robot.py --ros-args -p joint_rate:=100.0

Publishes:
    /joint_states        sensor_msgs/JointState   50 Hz  (try: ros2 topic hz /joint_states)
    /camera/image_raw    sensor_msgs/Image        30 Hz  (try: ros2 topic bw /camera/image_raw)
Subscribes:
    /cmd_vel             geometry_msgs/Twist             (the policy's commands)

Parameters:
    joint_rate    publish rate of /joint_states in Hz
    best_effort   publish /joint_states with BEST_EFFORT QoS (see the QoS exercise)
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "gripper"]
WIDTH, HEIGHT = 64, 48  # tiny image so echo doesn't flood the terminal


class FakeRobot(Node):
    def __init__(self):
        super().__init__("fake_robot")
        joint_rate = self.declare_parameter("joint_rate", 50.0).value
        best_effort = self.declare_parameter("best_effort", False).value

        reliability = ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE
        self.joint_pub = self.create_publisher(
            JointState, "joint_states", QoSProfile(depth=10, reliability=reliability))
        # Cameras conventionally use the "sensor data" profile (best effort, small queue).
        self.image_pub = self.create_publisher(Image, "camera/image_raw", qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

        self.positions = [0.0] * len(JOINTS)
        self.velocity = 0.0
        self.frame = 0
        self.create_timer(1.0 / joint_rate, self.publish_joints)
        self.create_timer(1.0 / 30.0, self.publish_image)
        self.get_logger().info(
            f"publishing /joint_states at {joint_rate} Hz ({reliability.name}), /camera/image_raw at 30 Hz")

    def on_cmd_vel(self, msg):
        # Pretend linear.x drives every joint, just so commands visibly change the state.
        self.velocity = msg.linear.x

    def publish_joints(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        for i in range(len(JOINTS)):
            self.positions[i] += self.velocity * 0.01
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [p + 0.1 * math.sin(t + i) for i, p in enumerate(self.positions)]
        msg.velocity = [self.velocity] * len(JOINTS)
        self.joint_pub.publish(msg)

    def publish_image(self):
        self.frame += 1
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height, msg.width = HEIGHT, WIDTH
        msg.encoding = "rgb8"
        msg.step = WIDTH * 3
        msg.data = bytes([self.frame % 256]) * (msg.step * HEIGHT)
        self.image_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
