#!/usr/bin/env python3
"""serial_bridge.py — мост между ROS 2 и Arduino для манипулятора.

Публикует /joint_states (текущее положение), принимает /target_joint_states
(целевое положение). Пересчёт q <-> угол сервы делает именно он, прошивка
оперирует только градусами серв.

Протокол — в firmware/minipulator/minipulator.ino.

Запуск:
    source /opt/ros/jazzy/setup.bash
    ros2 run minipulator serial_bridge --port /dev/ttyUSB0

    ros2 run minipulator serial_bridge --list-ports   # какие порты есть
"""

import argparse
import glob
import math
import sys
import termios
import time
import traceback

import serial
from serial.tools import list_ports

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

# ------------------------------------------------------------------ модель
# Всё ниже должно совпадать с robot.urdf.

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_joint"]

SERVO_ZERO = [90.0, 90.0, 10.0, 90.0, 0.0, 0.0]   # угол сервы в нуле модели
SERVO_SIGN = [1.0, -1.0, 1.0, -1.0, 1.0, 1.0]      # знак оси
SERVO_MIN = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SERVO_MAX = [180.0, 180.0, 90.0, 180.0, 180.0, 90.0]

START = [90.0, 90.0, 0.0, 40.0, 0.0, 0.0]          # стартовая поза, градусы серв

DEG = math.pi / 180.0


def servo_to_q(i: int, servo_deg: float) -> float:
    """Угол сервы (градусы) -> угол сустава (радианы)."""
    return SERVO_SIGN[i] * (servo_deg - SERVO_ZERO[i]) * DEG


def q_to_servo(i: int, q_rad: float) -> float:
    """Угол сустава (радианы) -> угол сервы (градусы), с ограничением."""
    servo = SERVO_ZERO[i] + SERVO_SIGN[i] * (q_rad / DEG)
    return max(SERVO_MIN[i], min(SERVO_MAX[i], servo))


class SerialBridge(Node):
    def __init__(self, ser: serial.Serial, speed: float, rate: float):
        super().__init__("serial_bridge")

        self.speed = speed
        self.ser = ser
        # Иначе ядро опускает DTR при закрытии порта: плата перезагружается и
        # 1.5 с стоит без импульсов — рука под весом клюёт вниз.
        try:
            attrs = termios.tcgetattr(self.ser.fileno())
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(self.ser.fileno(), termios.TCSANOW, attrs)
        except (OSError, termios.error):
            pass
        self.get_logger().info(f"порт {ser.port} открыт, {ser.baudrate} бод")

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.sub = self.create_subscription(
            JointState, "/target_joint_states", self.on_target, 10
        )

        self.buf = b""
        self.last_servo = None      # последнее известное положение серв, градусы
        self.pending = None         # цели, ждущие отправки
        self.ready = False
        self.publish_ok = False     # состояние молчит, пока рука не в стартовой позе
        self.last_rx = time.monotonic()
        self.warned = False         # уже пожаловались на тишину в порту
        self.last_msg = ""          # последнее сообщение платы, чтобы не дублировать

        self._startup()
        self.publish_ok = True
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

    # ------------------------------------------------------------ приём

    def on_target(self, msg: JointState):
        """Целевые углы суставов -> цели для серв."""
        if not msg.name:
            return

        targets = list(self.last_servo) if self.last_servo else list(START)
        for name, pos in zip(msg.name, msg.position):
            if name in JOINTS:
                i = JOINTS.index(name)
                targets[i] = q_to_servo(i, pos)

        self.pending = targets

    # ------------------------------------------------------------ отправка

    def _send_targets(self, targets):
        # длительность перехода: по самой быстрой оси, но не резко
        delta = max(
            abs(targets[i] - (self.last_servo[i] if self.last_servo else targets[i]))
            for i in range(6)
        )
        dur_ms = int(max(100.0, delta / self.speed * 1000.0))

        line = "T" + f"{targets[0]:.1f}" + "".join(f" {v:.1f}" for v in targets[1:])
        line += f" {dur_ms}\n"
        self.ser.write(line.encode("ascii"))
        self.get_logger().debug(f"-> {line.strip()}")

    # ------------------------------------------------------------ старт

    def _listen(self, seconds):
        """Читать порт заданное время, не отдавая управление rclpy."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self._poll()
            time.sleep(0.02)

    def _startup(self):
        # Плата, которая уже работает, отвечает сразу, и сбрасывать её не надо:
        # при сбросе 1.5 с нет импульсов, и рука под весом клюёт вниз.
        self._listen(0.5)
        if self.last_servo is None:
            self.get_logger().info("плата молчит — сбрасываю по DTR")
            self.ser.dtr = False
            time.sleep(0.15)
            self.ser.dtr = True
            # обрывок строки до сброса не должен съесть READY
            self.buf = b""
            self.ser.reset_input_buffer()
            t0 = time.monotonic()
            while not self.ready and time.monotonic() - t0 < 6.0:
                self._poll()
                time.sleep(0.02)
            self._listen(1.0)       # сервы доезжают в стартовую позу
        else:
            self.get_logger().info("плата уже работает — сброса нет")

        if self.last_servo is None:
            self.get_logger().warning("плата не ответила, старт не подтверждён")
            return
        self.ready = True

        # Парковка в стартовую позу тем же темпом, что и обычные движения.
        delta = 0.0
        if self.last_servo:
            delta = max(abs(START[i] - self.last_servo[i]) for i in range(6))
        self.get_logger().info(f"в стартовую позу: {delta:.0f}°, {delta / self.speed:.1f} с")
        self._send_targets(list(START))
        self._listen(delta / self.speed + 0.5)

    # ------------------------------------------------------------ цикл

    def _poll(self):
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
            self.last_rx = time.monotonic()

        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            self._handle_line(raw.decode("utf-8", errors="replace").strip())

    def _handle_line(self, text):
        if not text:
            return

        if text == "READY":
            self.ready = True
            self.get_logger().info("плата готова (READY)")
            return
        if text == "PONG":
            self.get_logger().info("плата ответила на пинг")
            return
        if text.startswith("E "):
            # прошивка шлёт "timeout" после почти каждой команды (в loop() время
            # берётся до разбора), и одинаковые подряд заливали бы лог
            if text == self.last_msg:
                return
            self.last_msg = text
            if "timeout" in text:
                self.get_logger().info(f"плата: {text[2:]}")
            else:
                self.get_logger().warning(f"плата: {text[2:]}")
            return

        if text.startswith("S"):
            parts = text[1:].split()
            if len(parts) != 6:
                return
            try:
                self.last_servo = [float(x) for x in parts]
            except ValueError:
                return
            # плата шлёт S только когда работает, — это надёжнее READY
            self.ready = True
            return

    def _publish_state(self):
        if not self.publish_ok or self.last_servo is None:
            return
        # плата молчит — не выдаём старое положение за текущее: MoveIt должен
        # видеть, что состояния нет, а не планировать от устаревшего
        if time.monotonic() - self.last_rx > 1.0:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [servo_to_q(i, s) for i, s in enumerate(self.last_servo)]
        self.pub.publish(msg)

    def on_timer(self):
        try:
            self._poll()
            if time.monotonic() - self.last_rx > 6.0:
                if not self.warned:
                    self.warned = True
                    self.get_logger().warning("6 с без данных с платы: проверь порт и прошивку")
            else:
                self.warned = False
            # до READY плата грузится: не шлём ей цели, иначе обрывок строки
            # может быть принят прошивкой за команду движения
            if self.pending is not None and self.ready:
                targets, self.pending = self.pending, None
                self._send_targets(targets)
            self._publish_state()
        except serial.SerialException as e:
            self.get_logger().error(f"порт отвалился: {e}")
            self.timer.cancel()

    def close(self):
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception:
            pass


def pick_port():
    """Найти плату: /dev/minipulator от udev, иначе CH340. ttyS* не трогаем."""
    if glob.glob("/dev/minipulator"):
        return "/dev/minipulator"
    for p in list_ports.comports():
        text = f"{p.description} {p.hwid}".upper()
        if "CH340" in text or "USB-SERIAL" in text or "ARDUINO" in text:
            return p.device
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    return ports[0] if ports else None


def main():
    argv = remove_ros_args(args=sys.argv)[1:]
    parser = argparse.ArgumentParser(description="мост ROS 2 <-> Arduino")
    parser.add_argument("--port", default="", help="например /dev/ttyUSB0 или COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--speed", type=float, default=60.0, help="град/с для переходов")
    parser.add_argument("--rate", type=float, default=20.0, help="Гц опроса порта")
    parser.add_argument("--list-ports", action="store_true", help="показать порты и выйти")
    args = parser.parse_args(argv)

    if args.list_ports:
        for p in list_ports.comports():
            print(f"{p.device:20s} {p.description}")
        return 0

    port = args.port or pick_port()
    if not port:
        print("порт не найден. Доступные порты:", file=sys.stderr)
        for p in list_ports.comports():
            print(f"  {p.device}  {p.description}", file=sys.stderr)
        return 1

    # номер порта меняется при переподключении — если указанный пропал, берём найденный
    fallback = pick_port()
    candidates = [port] if not fallback or fallback == port else [port, fallback]
    ser = None
    error = None
    for candidate in candidates:
        if candidate != port:
            print(f"порт {port} недоступен ({error}), беру {candidate}", file=sys.stderr)
        try:
            # exclusive: второй мост на том же порту должен падать, а не делить
            # с этим байты от платы
            ser = serial.Serial(candidate, args.baud, timeout=0, exclusive=True)
            break
        except serial.SerialException as e:
            error = e
    if ser is None:
        print(f"не удалось открыть порт {candidates[-1]}: {error}", file=sys.stderr)
        if "Permission denied" in str(error):
            print("нет прав на порт: перелогинься (группа dialout) или sudo chmod 666 <порт>",
                  file=sys.stderr)
        elif "busy" in str(error).lower():
            print("порт занят другим мостом: pkill -f serial_bridge", file=sys.stderr)
        return 1

    rclpy.init()
    node = None
    try:
        node = SerialBridge(ser, args.speed, args.rate)
        rclpy.spin(node)
    except serial.SerialException as e:
        print(f"порт отвалился: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
