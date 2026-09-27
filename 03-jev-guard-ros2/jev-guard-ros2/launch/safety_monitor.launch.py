"""Launch the jev_guard safety monitor.

    ros2 launch jev_guard safety_monitor.launch.py
    ros2 launch jev_guard safety_monitor.launch.py use_mock:=true
    ros2 launch jev_guard safety_monitor.launch.py params_file:=/path/to/my_arm.yaml

The Jev API key is read from $TYPESAFE_API_KEY in the launching shell.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("jev_guard"), "config", "guard.yaml")

    params_file = LaunchConfiguration("params_file")
    use_mock = LaunchConfiguration("use_mock")
    tick_hz = LaunchConfiguration("tick_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "use_mock",
                default_value="false",
                description="Use local heuristics instead of the Jev API (no key needed).",
            ),
            DeclareLaunchArgument("tick_hz", default_value="10.0"),
            Node(
                package="jev_guard",
                executable="guard_node",
                name="jev_guard",
                output="screen",
                emulate_tty=True,
                parameters=[params_file, {"use_mock": use_mock, "tick_hz": tick_hz}],
                additional_env={"PYTHONUNBUFFERED": "1"},
            ),
        ]
    )
