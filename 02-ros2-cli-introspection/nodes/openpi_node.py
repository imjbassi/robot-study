"""A stand-in for a VLA policy node (think openpi): observations in, actions out.

    python3 openpi_node.py

Subscribes:
    /joint_states        sensor_msgs/JointState
    /camera/image_raw    sensor_msgs/Image
Publishes:
    /cmd_vel             geometry_msgs/Twist      10 Hz  (try: ros2 topic echo /cmd_vel)
Services:
    /openpi_node/reset   std_srvs/Trigger                (try: ros2 service call ...)

There's no model here -- the "policy" just drives joint 0 back and forth between
two limits. The point is the node's wiring, which `ros2 node info /openpi_node`
shows in one screen.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger

LIMIT = 0.5  # rad


class OpenPiNode(Node):
    def __init__(self):
        super().__init__("openpi_node")
        self.speed = self.declare_parameter("speed", 0.3).value

        # Default QoS is RELIABLE. If the robot publishes BEST_EFFORT, these two
        # can't connect -- nothing arrives and nothing errors. See the README.
        self.create_subscription(JointState, "joint_states", self.on_joints, 10)
        self.create_subscription(Image, "camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_service(Trigger, "~/reset", self.on_reset)  # ~ = inside this node's namespace
        self.create_timer(0.1, self.act)

        self.joint0 = None
        self.frames = 0
        self.direction = 1.0
        self.create_timer(5.0, self.report)

    def on_joints(self, msg):
        self.joint0 = msg.position[0]

    def on_image(self, msg):
        self.frames += 1

    def act(self):
        if self.joint0 is None:
            return  # no observation yet -> no action
        if self.joint0 > LIMIT:
            self.direction = -1.0
        elif self.joint0 < -LIMIT:
            self.direction = 1.0
        cmd = Twist()
        cmd.linear.x = self.direction * self.speed
        self.cmd_pub.publish(cmd)

    def report(self):
        if self.joint0 is None:
            self.get_logger().warn("no /joint_states received yet -- is the robot up? QoS compatible?")
        else:
            self.get_logger().info(f"joint0={self.joint0:+.2f} rad, {self.frames} camera frames in last 5 s")
        self.frames = 0

    def on_reset(self, request, response):
        self.direction = 1.0
        response.success = True
        response.message = "policy state reset"
        self.get_logger().info("reset requested")
        return response


def main():
    rclpy.init()
    node = OpenPiNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
