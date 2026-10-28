#!/usr/bin/env python3
"""demos.py — демонстрации манипулятора: именованные последовательности поз.

Позы заучены ползунками (q в радианах, joint1..joint5 + gripper_joint).
Каждый переход планирует MoveIt (/move_action) и исполняет
исполнитель траекторий через мост. Обратной связи нет: «отпустил» —
это просто разжатие губки, скрипт не знает, был ли шарик в ней.

Запуск (MoveIt поднят с execute:=true, ползунки закрыты):
    ros2 run minipulator demos --list
    ros2 run minipulator demos pick_place
    ros2 run minipulator demos hello --speed 0.5
"""

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
GRIP = ["gripper_joint"]

START = (0.0, 0.0, -0.1745, 0.8727, 0.0)
PICK = (0.0, -0.547, 1.226, 0.976, 0.0)
PLACE = (1.5708, 0.387, 1.320, 0.976, 0.0)
LIFT = (1.5708, -0.445, 0.938, 0.976, 0.0)

# (название шага, q для joint1..joint5, gripper_joint)
DEMOS = {
    "pick_place": [
        ("подъехал", PICK, 1.40),
        ("взял", PICK, 0.119),
        ("перенёс", PLACE, 0.119),
        ("отпустил", PLACE, 0.62),
        ("поднялся", LIFT, 0.62),
    ],
    "hello": [
        ("встал", [0.0, -0.547, 1.040, 0.872, 0.0], 1.257),
        ("вправо", [0.0, -0.547, 1.040, 0.872, 0.764], 1.257),
        ("влево", [0.0, -0.547, 1.040, 0.872, 0.0], 1.257),
        ("вправо", [0.569, -0.547, 1.040, 0.872, 0.0], 1.257),
        ("влево", [-0.569, -0.547, 1.040, 0.872, 0.0], 1.257),
        ("в центр", [0.0, -0.547, 1.040, 0.872, 0.0], 0.178),
        ("закрыл губку", [0.0, -0.547, 1.040, 0.872, 0.0], 1.257),
        ("закрыл губку", [0.0, -0.547, 1.040, 0.872, 0.0], 0.178),
        ("закрыл губку", [0.0, -0.547, 1.040, 0.872, 0.0], 1.257),
    ],
    "nod": [
        ("наклон", PICK, 0.20),
        ("прямо", START, 0.20),
        ("наклон", PICK, 0.20),
        ("прямо", START, 0.20),
    ],
}

PAUSE = 0.2   # с: пауза после шага (move() и так ждёт конца движения)
TIMEOUT = 60  # с: потолок на один переход


class Demo(Node):
    def __init__(self, speed):
        super().__init__("demos")
        self.speed = speed
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def move(self, group, joints, values):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.speed
        req.max_acceleration_scaling_factor = self.speed
        constraints = Constraints()
        for name, value in zip(joints, values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/move_action не отвечает — MoveIt запущен?")

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{group}: цель отклонена")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=TIMEOUT)
        if not result_future.done():
            raise RuntimeError(f"{group}: нет результата за {TIMEOUT} с")
        code = result_future.result().result.error_code.val
        if code != 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS
            raise RuntimeError(f"{group}: ошибка MoveIt {code}")


def run(node, steps, pause):
    prev_arm = prev_grip = None
    for step, (title, arm, grip) in enumerate(steps, 1):
        node.get_logger().info(f"шаг {step}: {title}")
        # открытую губку ставим до движения, закрытую — после
        if prev_grip is None or grip > prev_grip:
            node.move("gripper", GRIP, [grip])
        if arm != prev_arm:
            node.move("arm", ARM, arm)
        if prev_grip is not None and grip < prev_grip:
            node.move("gripper", GRIP, [grip])
        prev_arm, prev_grip = arm, grip
        time.sleep(pause)


def main():
    parser = argparse.ArgumentParser(description="демонстрации манипулятора")
    parser.add_argument("demo", nargs="?", default="pick_place", choices=sorted(DEMOS),
                        help="какую демо проиграть")
    parser.add_argument("--list", action="store_true", help="показать демо и выйти")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="доля от лимитов MoveIt (1.0 = ~57 °/с)")
    parser.add_argument("--pause", type=float, default=PAUSE, help="пауза после шага, с")
    args = parser.parse_args()

    if args.list:
        for name, steps in DEMOS.items():
            print(f"{name:12s} {len(steps)} шагов: " + ", ".join(s[0] for s in steps))
        return 0

    rclpy.init()
    node = Demo(args.speed)
    try:
        node.get_logger().info(f"демо {args.demo}")
        run(node, DEMOS[args.demo], args.pause)
        node.get_logger().info("готово")
    except RuntimeError as e:
        node.get_logger().error(str(e))
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
