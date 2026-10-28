#!/usr/bin/env python3
"""trajectory_executor.py — сервер FollowJointTrajectory для манипулятора.

MoveIt присылает сюда точки траектории, узел отдаёт их в /target_joint_states
в темпе траектории. Пересчёт q -> углы серв и потолок 30 °/с делает мост
(minipulator/serial_bridge.py), прошивка ограничивает 150 °/с.

Запуск с MoveIt:  ros2 launch minipulator moveit.launch.py execute:=true
Отдельно:         ros2 run minipulator trajectory_executor
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

JOINTS = {"joint1", "joint2", "joint3", "joint4", "joint5", "gripper_joint"}
TOL = 0.01     # рад: считаем, что доехали (MoveIt требует старт не дальше 0.01)
GRACE = 3.0    # с: ждём сверх длительности траектории, потом сдаёмся


def secs(duration):
    return duration.sec + duration.nanosec * 1e-9


class Executor(Node):
    def __init__(self):
        super().__init__("trajectory_executor")
        self.state = {}     # последние /joint_states от моста
        self.pub = self.create_publisher(JointState, "/target_joint_states", 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        # имя совпадает с moveit/moveit_controllers.yaml: контроллер + action_ns
        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info("исполнитель готов: /arm_controller/follow_joint_trajectory")

    def on_state(self, msg):
        self.state = dict(zip(msg.name, msg.position))

    def on_goal(self, goal):
        traj = goal.trajectory
        if not traj.points or not set(traj.joint_names) <= JOINTS:
            self.get_logger().error("отклоняю цель: нет точек или чужие суставы")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def send(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = list(positions)
        self.pub.publish(msg)

    def reached(self, names, target):
        return all(abs(self.state.get(n, 1e9) - p) < TOL for n, p in zip(names, target))

    def execute(self, goal_handle):
        names = list(goal_handle.request.trajectory.joint_names)
        points = list(goal_handle.request.trajectory.points)
        result = FollowJointTrajectory.Result()
        start = self.get_clock().now().nanoseconds
        end = secs(points[-1].time_from_start)
        sent = -1

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                self.get_logger().info("траектория отменена")
                return result

            elapsed = (self.get_clock().now().nanoseconds - start) * 1e-9
            i = sent
            while i + 1 < len(points) and secs(points[i + 1].time_from_start) <= elapsed:
                i += 1
            if i > sent:
                self.send(names, points[i].positions)
                sent = i

            if elapsed >= end and self.reached(names, points[-1].positions):
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.succeed()
                return result
            if elapsed > end + GRACE:
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                goal_handle.abort()
                self.get_logger().warning("не доехали до конца траектории, сдаюсь")
                return result

            time.sleep(0.05)


def main():
    rclpy.init()
    node = Executor()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
