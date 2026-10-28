"""Robot model in RViz with joint sliders. No hardware and no MoveIt."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

SHARE = Path(get_package_share_directory("minipulator"))


def generate_launch_description():
    robot_description = (SHARE / "urdf" / "robot.urdf").read_text(encoding="utf-8")

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
        ),
    ])
