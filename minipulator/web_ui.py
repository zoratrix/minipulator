#!/usr/bin/env python3
"""web_ui.py — веб-пульт манипулятора: ползунки, джойстик, 3D.

Сервер на стандартной библиотеке: отдаёт страницу и robot.urdf из share,
шлёт состояние в браузер через Server-Sent Events и принимает команды
POST /cmd.

- ползунки -> /target_joint_states напрямую (как joint_state_publisher_gui);
- джойстик -> /compute_ik в MoveIt (position_only) -> /target_joint_states.
Исполнитель и демо не участвуют; мост держит порт.

Запуск: ros2 launch minipulator web.launch.py   -> http://127.0.0.1:8080
Отдельно: ros2 run minipulator web_ui --port 8080
"""

import argparse
import collections
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

SHARE = Path(get_package_share_directory("minipulator"))
WEB = SHARE / "web"
ROBOT = SHARE / "urdf" / "robot.urdf"
DATA = Path.home() / ".minipulator"

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
TIP = "link5"        # точка, которой управляет джойстик
MAX_STEP = 0.15      # рад: скачок больше — считаем ветвью IK и пропускаем

START = {"joint1": 0.0, "joint2": 0.0, "joint3": -0.17453292519943295,
         "joint4": 0.8726646259971648, "joint5": 0.0}          # парковка, как у моста
GRIP_OPEN = 1.2      # ~69°: не до упора
GRIP_CLOSE = 0.3     # ~17°: не до упора
SPEED_DEFAULT = 0.4  # доля от лимитов MoveIt (~23 °/с)


def clean_program(program):
    """Проверить и нормализовать программу: список шагов с 5 углами, губкой и паузой."""
    steps = program.get("steps") if isinstance(program, dict) else None
    if not isinstance(steps, list) or not steps:
        raise ValueError("нет шагов")
    out = []
    for step in steps:
        if isinstance(step, dict) and step.get("type") == "wait":
            out.append({
                "type": "wait",
                "name": str(step.get("name") or "пауза"),
                "ms": int(max(50, min(60000, float(step.get("ms", 1000))))),
            })
            continue
        joints = step.get("joints") if isinstance(step, dict) else None
        if not isinstance(joints, list) or len(joints) != 5:
            raise ValueError("в шаге нужно 5 углов")
        out.append({
            "type": "pose",
            "name": str(step.get("name") or f"pos_{len(out) + 1}"),
            "joints": [float(v) for v in joints],
            "gripper": float(step.get("gripper", GRIP_CLOSE)),
            "pause": max(0.0, min(10.0, float(step.get("pause", 0.3)))),
            "speed": max(0.05, min(1.0, float(step.get("speed", SPEED_DEFAULT)))),
        })
    return {"version": 1, "name": str(program.get("name", "program")),
            "description": str(program.get("description", ""))[:200], "steps": out}


class WebNode(Node):
    LIBRARY = DATA / "library.json"

    def __init__(self):
        super().__init__("web_ui")
        self.lock = threading.Lock()
        self.ik_lock = threading.Lock()
        self.state = {}
        self.logs = collections.deque(maxlen=400)
        self.log_seq = 0
        self.log_lock = threading.Lock()
        self.last_ik_log = 0.0
        self.running = False
        self.run_index = 0
        self.run_total = 0
        self.stop_flag = threading.Event()
        self.run_lock = threading.Lock()
        self.run_error = ""
        self.run_name = ""
        self.current_goal = None
        self.pub = self.create_publisher(JointState, "/target_joint_states", 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.create_subscription(Log, "/rosout", self.on_log, 50)
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.move = ActionClient(self, MoveGroup, "/move_action")

    def on_state(self, msg):
        with self.lock:
            self.state = dict(zip(msg.name, msg.position))

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def send(self, values):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(values)
        msg.position = [float(v) for v in values.values()]
        self.pub.publish(msg)

    def publish_joint(self, name, value):
        self.send({name: value})

    def stop(self):
        state = self.snapshot()
        if state:
            self.send(state)
            self.get_logger().info("стоп: держу текущую позу")

    def home(self):
        self.send(START)
        self.get_logger().info("домой: парковка в стартовую позу")

    def grip(self, name):
        value = GRIP_OPEN if name == "open" else GRIP_CLOSE
        self.publish_joint("gripper_joint", value)
        self.get_logger().info(f"захват: {'открыт' if name == 'open' else 'закрыт'} ({value:.2f} рад)")

    # ---------------------------------------------------------- программа

    def save_program(self, path, program):
        data = clean_program(program)
        p = Path(path or "program.json").expanduser()
        if not p.is_absolute():
            p = DATA / "programs" / p
        if p.suffix != ".json":
            p = p.with_suffix(".json")
        p.parent.mkdir(parents=True, exist_ok=True)
        data["name"] = p.stem
        data["saved"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        p.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        self.library_add(p, data)
        self.get_logger().info(f"программа сохранена: {p}")
        return {"ok": True, "path": str(p)}

    def load_program(self, path):
        if not path:
            return {"ok": False, "error": "путь не указан"}
        p = Path(path).expanduser()
        if not p.is_absolute():
            p = DATA / "programs" / p
        if not p.exists():
            return {"ok": False, "error": f"файл не найден: {p}"}
        try:
            data = clean_program(json.loads(p.read_text(encoding="utf-8")))
        except (ValueError, json.JSONDecodeError) as e:
            return {"ok": False, "error": f"плохой файл: {e}"}
        self.library_add(p, data)
        self.get_logger().info(f"программа загружена: {p}")
        return {"ok": True, "program": data}

    def start_program(self, steps, loop=False):
        try:
            data = clean_program({"steps": steps})
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        with self.run_lock:
            if self.running:
                return {"ok": False, "error": "уже выполняется"}
            self.running = True
            self.run_total = len(data["steps"])
            self.run_index = 0
            self.run_name = ""
            self.run_error = ""
            self.stop_flag.clear()
        self.get_logger().info(f"программа: {self.run_total} шагов"
                               + (" (по кругу)" if loop else ""))
        threading.Thread(target=self._run_program, args=(data["steps"], loop),
                         daemon=True).start()
        return {"ok": True, "total": self.run_total}

    def stop_program(self):
        self.stop_flag.set()
        goal = self.current_goal
        if goal is not None:
            goal.cancel_goal_async()          # прервать текущий переход MoveIt
        self.stop()
        self.get_logger().info("программа остановлена")

    def program_status(self):
        with self.run_lock:
            return {"running": self.running, "index": self.run_index,
                    "total": self.run_total, "name": self.run_name,
                    "error": self.run_error}

    @staticmethod
    def _wait(future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return done.wait(timeout)

    def _goal(self, group, joints, speed, plan_only=False):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = float(speed)
        req.max_acceleration_scaling_factor = float(speed)
        constraints = Constraints()
        for name, value in joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = plan_only
        return goal

    def plan_step(self, step):
        """Только планирование: вернуть точки траектории для предпросмотра."""
        try:
            cleaned = clean_program({"steps": [step]})["steps"][0]
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        if cleaned.get("type") != "pose":
            return {"ok": False, "error": "у паузы нет траектории"}
        if not self.move.wait_for_server(timeout_sec=5.0):
            return {"ok": False, "error": "move_group не отвечает"}
        goal = self._goal("arm", dict(zip(ARM, cleaned["joints"])), cleaned["speed"], True)
        future = self.move.send_goal_async(goal)
        if not self._wait(future, 10.0):
            return {"ok": False, "error": "MoveIt: нет ответа на цель"}
        handle = future.result()
        if handle is None or not handle.accepted:
            return {"ok": False, "error": "MoveIt отклонил цель"}
        result_future = handle.get_result_async()
        if not self._wait(result_future, 15.0):
            return {"ok": False, "error": "MoveIt: нет плана за 15 с"}
        res = result_future.result()
        code = res.result.error_code.val if res else -1
        if code != 1:
            return {"ok": False, "error": f"MoveIt: ошибка {code}"}
        traj = res.result.planned_trajectory.joint_trajectory
        names = list(traj.joint_names)
        points = [list(p.positions) for p in traj.points]
        if not points:
            return {"ok": False, "error": "пустая траектория"}
        points = points[::max(1, len(points) // 200)]
        return {"ok": True, "names": names, "points": points}

    def _move_group(self, group, joints, speed):
        """Спланировать и выполнить цель через MoveIt. Возвращает True/False."""
        if not self.move.wait_for_server(timeout_sec=5.0):
            self.run_error = "move_group не отвечает"
            return False
        goal = self._goal(group, joints, speed)

        future = self.move.send_goal_async(goal)
        if not self._wait(future, 10.0):
            self.run_error = "MoveIt: нет ответа на цель"
            return False
        handle = future.result()
        if handle is None or not handle.accepted:
            self.run_error = "MoveIt отклонил цель"
            return False

        self.current_goal = handle
        result_future = handle.get_result_async()
        if not self._wait(result_future, 120.0):
            self.run_error = "MoveIt: нет результата за 120 с"
            return False
        self.current_goal = None
        res = result_future.result()
        code = res.result.error_code.val if res else -1
        if code != 1:                      # moveit_msgs/MoveItErrorCodes.SUCCESS
            if self.stop_flag.is_set():
                return False               # отменили сами — это не ошибка
            self.run_error = f"MoveIt: ошибка {code}"
            return False
        return True

    def _run_program(self, steps, loop=False):
        try:
            while True:
                for i, step in enumerate(steps, 1):
                    if self.stop_flag.is_set():
                        return
                    with self.run_lock:
                        self.run_index = i
                        self.run_name = step.get("name", "")
                    self.get_logger().info(f"точка {i}/{len(steps)}: {self.run_name}")
                    if step.get("type") == "wait":
                        end = time.monotonic() + step["ms"] / 1000.0
                        while time.monotonic() < end:
                            if self.stop_flag.is_set():
                                return
                            time.sleep(0.05)
                        continue
                    # рука — планировщиком MoveIt (проверка столкновений и время)
                    if not self._move_group("arm", dict(zip(ARM, step["joints"])), step["speed"]):
                        return
                    # губка — отдельной группой, если отличается от текущей
                    state = self.snapshot()
                    if abs(state.get("gripper_joint", step["gripper"]) - step["gripper"]) > 0.02:
                        if not self._move_group("gripper", {"gripper_joint": step["gripper"]},
                                                step["speed"]):
                            return
                    end = time.monotonic() + step["pause"]
                    while time.monotonic() < end:
                        if self.stop_flag.is_set():
                            return
                        time.sleep(0.05)
                if not loop:
                    break
            self.get_logger().info("программа выполнена")
        finally:
            self.current_goal = None
            with self.run_lock:
                self.running = False
                self.run_index = 0
                self.run_name = ""

    def _library_read(self):
        try:
            items = json.loads(self.LIBRARY.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return []
        return [e for e in items if isinstance(e, dict)
                and e.get("path") and Path(e["path"]).exists()]

    def _library_write(self, items):
        self.LIBRARY.parent.mkdir(parents=True, exist_ok=True)
        self.LIBRARY.write_text(json.dumps(items[:30], ensure_ascii=False, indent=2),
                                encoding="utf-8")

    def library_add(self, path, data):
        items = [e for e in self._library_read() if e.get("path") != str(path)]
        items.insert(0, {"path": str(path), "name": data.get("name") or path.stem,
                         "description": data.get("description", ""),
                         "saved": data.get("saved", "")})
        self._library_write(items)

    def library_list(self):
        return self._library_read()

    def library_forget(self, path):
        self._library_write([e for e in self._library_read() if e.get("path") != path])
        return {"ok": True}

    def list_dir(self, path):
        """Содержимое папки для окна сохранения: подпапки и .json."""
        p = Path(path).expanduser() if path else DATA / "programs"
        if not p.is_absolute():
            p = DATA / "programs" / p
        if p.is_file():
            p = p.parent
        if not p.is_dir():
            return {"ok": False, "error": f"нет папки: {p}"}
        dirs, files = [], []
        try:
            for child in sorted(p.iterdir(), key=lambda c: c.name.lower()):
                if child.name.startswith("."):
                    continue
                try:
                    if child.is_dir():
                        dirs.append({"name": child.name, "path": str(child)})
                    elif child.suffix == ".json":
                        files.append({"name": child.name, "path": str(child)})
                except OSError:
                    continue
        except PermissionError:
            return {"ok": False, "error": f"нет доступа: {p}"}
        return {"ok": True, "path": str(p), "parent": str(p.parent),
                "dirs": dirs, "files": files[:100]}

    def on_log(self, msg):
        with self.log_lock:
            self.log_seq += 1
            self.logs.append({
                "seq": self.log_seq,
                "t": time.strftime("%H:%M:%S", time.localtime(msg.stamp.sec)),
                "level": msg.level,
                "name": msg.name,
                "msg": msg.msg,
            })

    def logs_since(self, since):
        with self.log_lock:
            return [e for e in self.logs if e["seq"] > since]

    def ik_pose(self, x, y, z):
        if not self.ik.service_is_ready():
            return False
        if not self.ik_lock.acquire(blocking=False):
            return False            # предыдущий IK ещё считается
        try:
            state = self.snapshot()
            if not state:
                return False
            req = GetPositionIK.Request()
            r = req.ik_request
            r.group_name = "arm"
            r.ik_link_name = TIP
            r.avoid_collisions = False
            r.timeout = Duration(seconds=0.2).to_msg()
            r.robot_state.joint_state.name = list(state)
            r.robot_state.joint_state.position = [state[n] for n in state]
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0      # position_only_ik ориентацию игнорирует
            r.pose_stamped = pose

            future = self.ik.call_async(req)
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(1.0):
                return False
            res = future.result()
            if res is None or res.error_code.val != 1:
                now = time.monotonic()
                if now - self.last_ik_log > 1.0:      # не флудим в лог
                    self.last_ik_log = now
                    self.get_logger().warning("IK: решения нет")
                return False
            sol = dict(zip(res.solution.joint_state.name,
                           res.solution.joint_state.position))
            if any(abs(sol.get(n, 0.0) - state.get(n, 0.0)) > MAX_STEP for n in ARM):
                self.get_logger().warning("IK дал скачок больше 0.15 рад — пропускаю")
                return False
            self.send({n: sol[n] for n in ARM if n in sol})
            return True
        finally:
            self.ik_lock.release()


class Handler(BaseHTTPRequestHandler):
    node = None     # WebNode, ставится перед стартом сервера

    def log_message(self, *args):
        pass

    def _file(self, path, ctype):
        try:
            data = path.read_bytes()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            return self._file(WEB / "index.html", "text/html; charset=utf-8")
        if self.path == "/app.js":
            return self._file(WEB / "app.js", "text/javascript; charset=utf-8")
        if self.path == "/style.css":
            return self._file(WEB / "style.css", "text/css; charset=utf-8")
        if self.path.startswith("/vendor/"):
            rel = Path(self.path[len("/vendor/"):])
            target = WEB / "vendor" / rel
            if rel.is_absolute() or ".." in rel.parts or not target.is_file():
                return self.send_error(404)
            ctype = ("text/javascript; charset=utf-8" if target.suffix == ".js"
                     else "text/plain; charset=utf-8")
            return self._file(target, ctype)
        if self.path == "/robot.urdf":
            return self._file(ROBOT, "text/plain; charset=utf-8")
        if self.path == "/events":
            return self._events()
        if urlparse(self.path).path == "/logs":
            return self._logs()
        if urlparse(self.path).path == "/program/status":
            return self._json(self.node.program_status())
        if urlparse(self.path).path == "/library":
            return self._json(self.node.library_list())
        if urlparse(self.path).path == "/fs":
            query = parse_qs(urlparse(self.path).query)
            return self._json(self.node.list_dir(query.get("path", [""])[0]))
        if urlparse(self.path).path == "/program/load":
            query = parse_qs(urlparse(self.path).query)
            return self._json(self.node.load_program(query.get("path", [""])[0]))
        self.send_error(404)

    def _logs(self):
        query = parse_qs(urlparse(self.path).query)
        since = int(query.get("since", ["0"])[0])
        body = json.dumps(self.node.logs_since(since), ensure_ascii=False).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            while True:
                state = self.node.snapshot()
                body = json.dumps({"names": list(state),
                                   "positions": [state[n] for n in state]})
                self.wfile.write(f"data: {body}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _read_json(self):
        length = int(self.headers.get("Content-Length", 0))
        try:
            return json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            return {}

    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _cmd(self, cmd):
        kind = cmd.get("type")
        try:
            if kind in ("joint", "pose") and self.node.running:
                return {"ok": False, "error": "идёт программа"}
            if kind == "joint":
                self.node.publish_joint(cmd["name"], float(cmd["value"]))
                return {"ok": True}
            if kind == "pose":
                return {"ok": self.node.ik_pose(cmd["x"], cmd["y"], cmd["z"])}
            if kind == "stop":
                self.node.stop()
                return {"ok": True}
            if kind == "home":
                self.node.home()
                return {"ok": True}
            if kind == "grip":
                self.node.grip(cmd.get("name", "open"))
                return {"ok": True}
        except Exception as e:  # чтобы браузер всегда получал ответ
            self.node.get_logger().error(f"команда {kind}: {e}")
        return {"ok": False}

    def do_POST(self):
        path = urlparse(self.path).path
        body = self._read_json()
        try:
            if path == "/cmd":
                return self._json(self._cmd(body))
            if path == "/program/save":
                return self._json(self.node.save_program(body.get("path", ""),
                                                        body.get("program", {})))
            if path == "/program/run":
                return self._json(self.node.start_program(body.get("steps", []),
                                                          bool(body.get("loop", False))))
            if path == "/program/stop":
                self.node.stop_program()
                return self._json({"ok": True})
            if path == "/program/plan":
                return self._json(self.node.plan_step(body.get("step", {})))
            if path == "/program/forget":
                return self._json(self.node.library_forget(body.get("path", "")))
        except Exception as e:
            return self._json({"ok": False, "error": str(e)}, 400)
        self.send_error(404)


def main():
    parser = argparse.ArgumentParser(description="веб-пульт манипулятора")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    # launch передаёт --ros-args -r __node:=...: argparse о них знать не должен,
    # rclpy.init() ниже разберёт их из sys.argv сам
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    (DATA / "programs").mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = WebNode()
    Handler.node = node
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"веб-пульт: http://{args.host}:{args.port}")
    try:
        rclpy.spin(node, MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
