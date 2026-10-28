"""MoveIt 2 bringup: robot state publisher, serial bridge, move_group and RViz.

Plan only by default. Pass execute:=true to start the FollowJointTrajectory
executor as well; RViz's Execute button and MoveIt action clients then drive
the arm.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = Path(get_package_share_directory("minipulator"))


def load(name):
    return yaml.safe_load((SHARE / "config" / name).read_text(encoding="utf-8"))


def generate_launch_description():
    robot_description = (SHARE / "urdf" / "robot.urdf").read_text(encoding="utf-8")
    robot_description_semantic = (SHARE / "config" / "robot.srdf").read_text(encoding="utf-8")

    planning = load("joint_limits.yaml")
    planning["default_velocity_scaling_factor"] = 0.6
    planning["default_acceleration_scaling_factor"] = 0.6

    moveit = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": load("kinematics.yaml"),
        "robot_description_planning": planning,
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": load("ompl_planning.yaml"),
        **load("moveit_controllers.yaml"),
    }

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/minipulator",
                              description="Serial port of the Arduino board"),
        DeclareLaunchArgument("execute", default_value="false",
                              description="Start the FollowJointTrajectory executor"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
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
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[moveit],
            output="screen",
        ),
        Node(
            package="minipulator",
            executable="trajectory_executor",
            name="trajectory_executor",
            condition=IfCondition(LaunchConfiguration("execute")),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(SHARE / "rviz" / "moveit.rviz")],
            output="screen",
        ),
    ])
