"""Joint sliders without MoveIt.

joint_state_publisher_gui publishes /target_joint_states, the bridge converts the
angles to servo commands and publishes /joint_states back for RViz.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = Path(get_package_share_directory("minipulator"))

START_Q = {
    "joint1": 0.0,
    "joint2": 0.0,
    "joint3": -0.17453292519943295,
    "joint4": 0.8726646259971648,
    "joint5": 0.0,
    "gripper_joint": 0.0,
}


def generate_launch_description():
    robot_description = (SHARE / "urdf" / "robot.urdf").read_text(encoding="utf-8")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/minipulator",
                              description="Serial port of the Arduino board"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[{"zeros": START_Q}],
            remappings=[("/joint_states", "/target_joint_states")],
            output="screen",
        ),
        Node(
            package="minipulator",
            executable="serial_bridge",
            name="serial_bridge",
            arguments=["--port", LaunchConfiguration("port")],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(SHARE / "rviz" / "teleop.rviz")],
            output="screen",
        ),
    ])
