# minipulator

ROS 2 Jazzy package for a 6-servo desktop manipulator controlled by an Arduino
Nano over USB serial. The servos have no feedback. Motion planning is done in
MoveIt. The robot is operated through a web UI: manual control, creating,
saving and running programs.

Русская версия этого README: [`../README.md`](../README.md).

## Hardware

| Part | Notes |
| --- | --- |
| 5 arm servos + 1 gripper servo | TD-8120MG, 180° range |
| Arduino Nano (ATmega328P) | CH340 USB bridge, VID `1a86`, PID `7523` |
| Servo power supply | 5–6 V, sized for the stall current of all servos; common ground with the Nano |
| PC | Ubuntu 24.04 with ROS 2 Jazzy |

Servo signal pins are D4–D9 (arm joints 1–5, then the gripper).

## Features

- Motion planning through MoveIt 2: OMPL, KDL, position-only IK.
- Communication with the low-level board (Arduino Nano) over a serial port.
- Web UI with manual control, 3D model, program editor and logs.
- `FollowJointTrajectory` executor for MoveIt and programs.

## Repository layout

```
minipulator/
├── config/                  MoveIt parameters (SRDF, IK, limits, OMPL, controllers)
├── firmware/minipulator/    Arduino sketch
├── instructions/            adapting the package to another robot
├── launch/                  bringup files
├── minipulator/             Python nodes
├── resource/                ament index marker
├── rviz/                    RViz configurations
├── udev/                    udev rule
├── urdf/robot.urdf          robot model
└── web/                     web UI assets
```

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy.
- MoveIt 2, `robot_state_publisher`, `joint_state_publisher_gui`, RViz 2.
- `python3-serial`, `python3-yaml`.

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2 \
  python3-serial python3-yaml
```

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repository> minipulator
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## udev rule

A CH340 board can get a different device number each time it is plugged in
(`/dev/ttyUSB0`, `/dev/ttyUSB1`), and access depends on the `dialout` group.
The rule creates a stable `/dev/minipulator` with 0666 permissions, so the
launch commands and the bridge do not depend on the number.

Apply it once after installing the package, before the first run with the board.
If the port disappears after replugging, check the rule.

```bash
sudo cp "$(ros2 pkg prefix minipulator)/share/minipulator/udev/99-minipulator.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Replug the board. Without the rule the bridge still finds the first `ttyUSB*`
device; pass a port explicitly if several boards are connected:

```bash
ros2 launch minipulator teleop.launch.py port:=/dev/ttyUSB0
```

## Firmware

Open `firmware/minipulator/minipulator.ino` in the Arduino IDE, select the
**Arduino Nano** board (some boards require the **ATmega328P (Old Bootloader)**
bootloader) and upload. The sketch uses only the `Servo` library.

On power-up the board waits 1.5 s, attaches the servos one by one (250 ms
apart) and moves to the start pose.

Serial protocol (115200 baud, ASCII lines, `\n`):

| Direction | Message | Meaning |
| --- | --- | --- |
| PC → board | `T<s1> <s2> <s3> <s4> <s5> <s6> <ms>` | target servo angles and transition time |
| PC → board | `P` | ping |
| board → PC | `READY` | boot finished, start pose reached |
| board → PC | `S<s1> ... <s6>` | current servo angles, 20 Hz |
| board → PC | `PONG` | ping reply |
| board → PC | `E <text>` | error message |

## Running

The serial bridge takes the port; run one bringup at a time.

### Model only

```bash
ros2 launch minipulator view.launch.py
```

Starts `robot_state_publisher`, `joint_state_publisher_gui` and RViz. No
hardware is touched.

### Joint sliders

```bash
ros2 launch minipulator teleop.launch.py
```

Direct servo control with the sliders: the chosen positions are published to
`/target_joint_states`, the bridge moves the servos and publishes
`/joint_states` for RViz.

### MoveIt in RViz

```bash
ros2 launch minipulator moveit.launch.py execute:=true
```

Starts `robot_state_publisher`, the bridge, `move_group` and RViz. With
`execute:=true` the trajectory executor is started too, so **Plan → Execute**
works. Without it only planning is available.

### Web UI

```bash
ros2 launch minipulator web.launch.py
# http://127.0.0.1:8080
```

Starts `robot_state_publisher`, the bridge, `move_group`, the executor and the
web server. Arguments: `port:=/dev/ttyUSB0`, `web_port:=8081`,
`host:=0.0.0.0` (exposes the page on the network; there is no authentication).

three.js is bundled in `web/vendor` under its MIT license, so the page works offline.

### Demos

With a MoveIt bringup running (`execute:=true`):

```bash
ros2 run minipulator demos --list
ros2 run minipulator demos pick_place
ros2 run minipulator demos hello --speed 0.5 --pause 0.2
```

## Web UI

### Programming the robot

A program is a list of steps:

- `pose` — link rotation angles (rad), gripper opening angle, pause after
  reaching the point (s) and speed (0.1–1.0 of the MoveIt limits).
- `wait` — a standalone pause in milliseconds; the robot holds its pose.

Each step has a name, editable in the list. `▶` runs one step, `👁` plans it
without moving and draws the wrist path, `⧉` duplicates it, `↑`/`↓` reorder,
`⟲` replaces the pose with the current one, `✕` deletes. The loop checkbox
repeats the program until STOP. The pause step is added with `⏸ Пауза`.

Programs are executed through MoveIt `/move_action` (planning and collision
checks) and the trajectory executor. Manual controls are blocked while a
program runs.

Program JSON:

```json
{
  "version": 1,
  "name": "pick_place",
  "description": "pick the ball and put it aside",
  "saved": "2026-10-28T18:00:00",
  "steps": [
    {"type": "pose", "name": "pos_1", "joints": [0.0, 0.0, -0.1745, 0.8727, 0.0],
     "gripper": 0.3, "pause": 0.3, "speed": 0.4},
    {"type": "wait", "name": "wait_2", "ms": 1000}
  ]
}
```

## Joint conventions

`joint1`…`joint5` are the robot links, `gripper_joint` drives the gripper jaw.
The bridge converts between model angles and servo angles:

```
q_rad      = sign * (servo_deg - zero_deg) * pi / 180
servo_deg  = zero_deg + sign * q_rad * 180 / pi
```

`zero_deg = [90, 90, 10, 90, 0, 0]`, `sign = [+1, -1, +1, -1, +1, +1]`,
start pose `q = [0, 0, -0.1745, 0.8727, 0]` → servo `[90, 90, 0, 40, 0, 0]`.

| Joint | Lower (rad) | Upper (rad) |
| --- | --- | --- |
| joint1 | -1.5708 | 1.5708 |
| joint2 | -1.5708 | 1.5708 |
| joint3 | -0.1746 | 1.3963 |
| joint4 | -1.5708 | 1.5708 |
| joint5 | 0.0 | 3.1416 |
| gripper_joint | 0.0 | 1.5708 |

Frames: `base_link` → `link1` … → `link5`; the gripper is attached to `link5`.
World axes: X to the right, Y away from the operator, Z up.

MoveIt limits are `1.0 rad/s` and `3.0 rad/s²` per joint in
`config/joint_limits.yaml`; the bridge defaults to 60 °/s and the firmware caps
transitions at 150 °/s and enforces a minimum move time.

## Safety

The arm is open loop. A servo that stalls, a blocked joint or a detached cable
is not detected by any node; `/joint_states` keep reporting the commanded
angles.

## Adapting to another robot

The package ships with the dimensions, limits and servo calibration of this
particular arm. To use another arm with the same architecture (Arduino + ROS 2,
open-loop servos, different sizes or URDF) follow
[`ADAPTING_TO_ANOTHER_ROBOT.md`](ADAPTING_TO_ANOTHER_ROBOT.md).

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `/dev/minipulator` missing | udev rule installed, board replugged |
| `Permission denied` on the port | user in `dialout`, or `sudo chmod 666 <port>` |
| `port busy` | another bridge is running: `pkill -f serial_bridge` |
| no `/joint_states` | bridge failed at start, see its output |
| MoveIt cannot plan | target out of reach, start state stale, collision between links |
| web page blank | open the browser console and check `/vendor/three.module.js` |
| IK fails in the joystick | target outside the workspace or joint limits |
| program stops with `MoveIt: ошибка` | the pose has no IK solution or collides |

## Limitations

- No encoders, no force sensing, no cameras.
- The URDF is built from boxes and does not account for inertia.
- The web server is a single-user tool without authentication.
