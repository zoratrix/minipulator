# Адаптация minipulator под другого робота

Этот пакет поставляется с геометрией, лимитами и калибровкой одной конкретной
руки. Ниже перечислены все места, где эта рука зашита, чтобы тот же код можно
было навести на похожую машину: Arduino (или совместимая плата) с PWM-сервами,
без обратной связи, ROS 2 на ПК.

Предполагаемая архитектура:

- 5 суставов руки плюс один серв захвата, сервы 180°;
- плата получает целевые углы серв и время перехода, отдаёт свои
  скомандованные углы; вся кинематика живёт на ПК;
- нет энкодеров, датчиков усилия и камер.

Если число суставов другое, изменений будет больше: список из шести суставов
зашит в нескольких файлах.

> English version: second half of this file.

## Поток данных

```
MoveIt / веб-интерфейс / ползунки
        │  /target_joint_states (рад)
        ▼
serial_bridge ── q → градусы серв ──► Arduino ──► сервы
        ▲                                   │
        └──── /joint_states (рад) ◄─────────┘  скомандованные углы, 20 Гц
```

## Чек-лист

| # | Файл | Что менять |
| --- | --- | --- |
| 1 | `urdf/robot.urdf` | дерево звеньев и суставов, размеры, оси, лимиты |
| 2 | `config/robot.srdf` | цепь группы, сустав губки, именованные позы, пары столкновений |
| 3 | `config/joint_limits.yaml` | скорость и ускорение на сустав |
| 4 | `config/kinematics.yaml` | обычно не меняется (`position_only_ik`) |
| 5 | `config/moveit_controllers.yaml` | список суставов `arm_controller` |
| 6 | `minipulator/serial_bridge.py` | порядок суставов, нули/знаки/диапазоны серв, стартовая поза, порт |
| 7 | `firmware/minipulator/minipulator.ino` | пины, диапазоны серв, стартовая поза, baud, скорость |
| 8 | `udev/99-minipulator.rules` | VID/PID USB и имя симлинка |
| 9 | `launch/*.launch.py` | порт по умолчанию, стартовая поза в `teleop` |
| 10 | `minipulator/web_ui.py` | имена руки и кончика, углы открытия/закрытия губки |
| 11 | `web/app.js` | имена руки и кончика, подписи суставов |
| 12 | `rviz/*.rviz` | fixed frame, группа планирования |
| 13 | `minipulator/demos.py` | заученные позы (по желанию) |
| 14 | `README.md` | описание железа (по желанию) |

## 1. URDF

`urdf/robot.urdf` — модель из коробок: у каждого звена есть визуальная коробка,
у каждого сустава — origin, axis и limits. Меши и инерция не используются,
MoveIt они не нужны.

- Заменить размеры коробок и значения `origin` у суставов на размеры новой
  руки. Origin — вектор от родительского сустава к дочернему вдоль звена.
- По возможности оставить имена суставов: `joint1`…`joint5` и
  `gripper_joint`. Они упоминаются в SRDF, конфигурации контроллера, мосте и
  веб-интерфейсе.
- Задать `axis` каждого сустава. Ошибка в оси сразу видна в RViz: сустав
  вращается вокруг не той линии.
- Задать `limit lower/upper` по механическому диапазону. У прошивки свои
  лимиты, значения должны совпадать.
- Оставить имя кончика `link5`: это цель IK и точка, которую рисует
  веб-интерфейс. Если имя меняется, поправить `TIP` в `minipulator/web_ui.py`
  и `web/app.js`, а также цепь в SRDF.
- `base_link` — неподвижное основание. Мировая система координат в RViz должна
  быть `base_link` (или его родителем).

## 2. SRDF

`config/robot.srdf` содержит:

- группу `arm`: цепь `base_link` → `link5`;
- группу `gripper`: сустав `gripper_joint`;
- именованные позы `start`, `turn`, `open`, `closed`;
- `disable_collisions` для соседних звеньев и для пары `gripper_finger` /
  `fixed_jaw` (губка в URDF — одно тело, поэтому модель видит постоянный
  контакт).

Менять концы цепи, если переименованы кончик или основание; править
именованные позы под новые углы; пересобирать `disable_collisions`, если
звенья добавились или исчезли. Обычно SRDF генерирует MoveIt Setup Assistant;
файл — обычный XML, его можно править руками.

## 3. Лимиты суставов

`config/joint_limits.yaml` хранит `max_velocity` и `max_acceleration` в рад/с
и рад/с². Штатные значения — 1.0 и 3.0. Они же задают 100 % скорости для
редактора программ и для масштабирования MoveIt.

## 4. IK и планирование

`config/kinematics.yaml` использует KDL с `position_only_ik: true`: это
позволяет 5-DOF руке доставать точки XYZ без полной ориентации. Менять не
нужно, если у новой руки столько же степеней свободы и не требуется другой
плагин IK.

`config/ompl_planning.yaml` содержит конвейер OMPL и адаптеры запросов и
ответов. От размеров робота он не зависит.

## 5. Контроллер

`config/moveit_controllers.yaml` объявляет один контроллер
`FollowJointTrajectory` с именем `arm_controller` и перечисляет все шесть
суставов. Если имена суставов другие, поправить список. Исполнитель отдаёт
действие `/arm_controller/follow_joint_trajectory`.

## 6. Мост

`minipulator/serial_bridge.py` содержит калибровку:

```python
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_joint"]
SERVO_ZERO = [90.0, 90.0, 10.0, 90.0, 0.0, 0.0]
SERVO_SIGN = [1.0, -1.0, 1.0, -1.0, 1.0, 1.0]
SERVO_MIN  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SERVO_MAX  = [180.0, 180.0, 90.0, 180.0, 180.0, 90.0]
START      = [90.0, 90.0, 0.0, 40.0, 0.0, 0.0]
```

- `JOINTS` — в том же порядке, что сервы на плате.
- `SERVO_ZERO` — угол сервы, при котором угол модели равен нулю. Это
  калибровка: поставить сустав в нулевое положение модели и посмотреть, какой
  угол отдаёт плата.
- `SERVO_SIGN` переворачивает сустав, если его положительное направление не
  совпадает с моделью.
- `SERVO_MIN` / `SERVO_MAX` ограничивают команды сервам. Должны совпадать с
  прошивкой.
- `START` — парковочная поза в градусах серв. Отправляется при старте и по
  кнопке «Домой» в веб-интерфейсе. Держать внутри механического диапазона.

Мост сам ищет порт: `/dev/minipulator` (udev), затем CH340 `ttyUSB*`. Если у
новой платы другой USB-мост, поправить `pick_port()` или передавать `--port`
явно. `--baud` (по умолчанию 115200), `--speed` (60 °/с) и `--rate` (20 Гц) —
параметры командной строки.

## 7. Прошивка

`firmware/minipulator/minipulator.ino`:

```cpp
const uint8_t AXES = 6;
const uint8_t PINS[AXES] = {4, 5, 6, 7, 8, 9};
const float SERVO_MIN[AXES] = {0, 0, 0, 0, 0, 0};
const float SERVO_MAX[AXES] = {180, 180, 90, 180, 180, 90};
const float START_ANGLE[AXES] = {90, 90, 0, 40, 0, 0};
const unsigned long BAUD = 115200;
const float MAX_SPEED = 150.0;
```

Заменить `PINS` на пины сервов, `SERVO_MIN`/`SERVO_MAX` — на безопасный
механический диапазон, `START_ANGLE` — на парковочную позу. `BAUD` должен
совпадать со скоростью моста. `MAX_SPEED` ограничивает скорость переходов,
`WATCHDOG_MS` удерживает момент, когда ПК перестал слать команды,
`REPORT_MS` — период отчёта о положении.

Протокол (`T`, `S`, `P`, `READY`, `PONG`, `E`) для похожей платы менять не
нужно. Если писать свою прошивку, эти сообщения лучше сохранить.

## 8. udev

`udev/99-minipulator.rules` ловит CH340 (`1a86:7523`) и создаёт
`/dev/minipulator`. Заменить VID/PID на данные новой платы (`lsusb`) и имя
симлинка, если нужно другое. После этого поправить `port` по умолчанию в
файлах запуска. Без правила передавать `port:=/dev/ttyUSB0`.

## 9. Файлы запуска

- `launch/moveit.launch.py`, `teleop.launch.py`, `web.launch.py`: порт по
  умолчанию — `/dev/minipulator`.
- `launch/teleop.launch.py`: `START_Q` должна совпадать со `START` моста в
  радианах. Ползунки стартуют оттуда, иначе первый же тик уведёт руку.
- `launch/web.launch.py`: значения `host` и `web_port` по умолчанию.

Все файлы запуска читают URDF и YAML MoveIt из share-каталога пакета, поэтому
других путей править не нужно.

## 10. Веб-сервер

`minipulator/web_ui.py`:

```python
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
TIP = "link5"
GRIP_OPEN = 1.2
GRIP_CLOSE = 0.3
MAX_STEP = 0.15
```

Обновить имена суставов и кончика, если они переименованы; углы открытия и
закрытия для другой губки; `MAX_STEP`, если IK новой руки скачет между
ветвями. Программы и библиотека лежат в `~/.minipulator` и от робота не
зависят.

## 11. Веб-клиент

`web/app.js`:

```js
const ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'];
const GRIP = 'gripper_joint';
const TIP = 'link5';
const LABEL = { joint1: 'j1', ... };
```

3D-модель строится из `urdf/robot.urdf` во время работы, лимиты берутся из
URDF, поэтому править нужно только имена и подписи. Заголовок страницы и
легенда осей — в `web/index.html`.

## 12. RViz

`rviz/moveit.rviz` и `rviz/teleop.rviz` хранят fixed frame, группу
MotionPlanning и топик с описанием робота. Если группа или frame меняются:
открыть RViz, поправить отображения и сохранить конфигурацию поверх файла.
Неверная группа в RViz — обычная причина того, что панель MotionPlanning не
показывает робота.

## 13. Демонстрации и программы

`minipulator/demos.py` содержит заученные позы для этой руки. Переучить их
можно веб-интерфейсом или ползунками, затем перенести углы. Сохранённые
программы — обычные JSON-файлы, их можно править руками; лежат они вне пакета.

## Проверка

Пройти список после изменений:

1. `ros2 launch minipulator view.launch.py` — модель совпадает с реальной
   рукой, каждый сустав вращается вокруг правильной оси.
2. `ros2 launch minipulator teleop.launch.py` — сдвинуть один сустав на
   несколько градусов. Если он едет не туда, перевернуть соответствующий
   `SERVO_SIGN`; если неверен нуль — поправить `SERVO_ZERO`.
3. Скомандовать нулевую позу (`q = 0`) и проверить физический нуль. Записать
   углы серв — это и есть калибровка `SERVO_ZERO`.
4. Скомандовать стартовую позу: она должна совпасть со `START` и `START_Q`.
5. Открыть веб-интерфейс: точка запястья в 3D должна совпадать с реальной
   рукой. Потянуть гизмо на 2 см — рука должна поехать плавно.
6. В RViz спланировать небольшое движение по суставам и выполнить. Провал
   здесь обычно означает устаревшее стартовое состояние (мост не запущен) или
   столкновение в URDF.
7. Подвести сустав к пределам: прошивка и мост должны ограничить команду, а
   не упереться.
8. Записать в веб-интерфейсе две точки, выполнить программу, нажать СТОП
   посреди движения.
9. Посмотреть вкладку логов: не появились ли `MoveIt: ошибка` и
   `IK: решения нет`.

## Заметки для похожего железа

- Диапазоны импульсов у сервов разные. `SERVO_MIN`/`SERVO_MAX` задавать по
  механическому диапазону, а не по номинальным 0–180°.
- Рука работает без обратной связи. После перезапуска питания или заклинивания
  реальное положение может отличаться от `/joint_states`; перед программами
  лучше вернуться в стартовую позу.
- Регулятор 5 В на Nano не тянет сервы. Нужен отдельный блок питания с общей
  землёй.
- Если плата не на CH340, поправить udev-правило или порт по умолчанию.
- Если у новой руки другое число суставов, предположение о шести суставах в
  мосте, контроллере, SRDF и веб-интерфейсе придётся менять согласованно.

---

# Adapting minipulator to another robot

This package ships with the geometry, joint limits and servo calibration of one
particular 6-servo arm. The instructions below describe every place that encodes
that robot, so the same code can drive a similar machine: an Arduino (or
compatible board) with PWM servos, no feedback, and ROS 2 on a PC.

The assumed architecture:

- 5 arm joints plus one gripper servo, 180° PWM servos.
- The board receives target servo angles and a transition time, and reports its
  own commanded angles back. All kinematics live on the PC.
- No encoders, no force sensors, no cameras.

If the number of joints differs, more code changes are needed than listed here;
the joints are a fixed list of six in several files.

## Data flow

```
MoveIt / web UI / sliders
        │  /target_joint_states (rad)
        ▼
serial_bridge ── q → servo degrees ──► Arduino ──► servos
        ▲                                   │
        └──── /joint_states (rad) ◄─────────┘  commanded angles, 20 Hz
```

## Checklist

| # | File | What to change |
| --- | --- | --- |
| 1 | `urdf/robot.urdf` | link/joint tree, dimensions, axes, limits |
| 2 | `config/robot.srdf` | group chain, gripper joint, named states, collision pairs |
| 3 | `config/joint_limits.yaml` | velocity and acceleration per joint |
| 4 | `config/kinematics.yaml` | usually unchanged (`position_only_ik`) |
| 5 | `config/moveit_controllers.yaml` | joint list of `arm_controller` |
| 6 | `minipulator/serial_bridge.py` | joint order, servo zero/sign/range, start pose, port |
| 7 | `firmware/minipulator/minipulator.ino` | pins, servo ranges, start pose, baud, speed |
| 8 | `udev/99-minipulator.rules` | USB VID/PID and symlink name |
| 9 | `launch/*.launch.py` | default serial port, start pose in `teleop` |
| 10 | `minipulator/web_ui.py` | arm/tip names, gripper open/close |
| 11 | `web/app.js` | arm/tip names, joint labels |
| 12 | `rviz/*.rviz` | fixed frame, planning group |
| 13 | `minipulator/demos.py` | taught poses (optional) |
| 14 | `README.md` | hardware description (optional) |

## 1. URDF

`urdf/robot.urdf` is a box model: every link has a box visual and every joint an
origin, axis and limits. No meshes and no inertials are used; MoveIt does not
need them.

- Replace the box sizes and the joint `origin` values with the dimensions of the
  new arm. Origins are measured from the parent joint to the child joint along
  the link.
- Keep the joint names if possible: `joint1`…`joint5` and `gripper_joint`. They
  are referenced in the SRDF, the controller configuration, the bridge and the
  web UI.
- Set each joint `axis`. A wrong axis shows up immediately in RViz as a joint
  rotating around the wrong line.
- Set `limit lower/upper` to the mechanical range. The firmware has its own
  limits; both must agree.
- Keep the tip link name `link5`. It is the IK target and the point drawn in the
  web UI. If you rename it, update `TIP` in `minipulator/web_ui.py` and
  `web/app.js`, and the SRDF chain.
- `base_link` is the fixed base. The world frame in RViz must be `base_link`
  (or a parent of it).

## 2. SRDF

`config/robot.srdf` contains:

- group `arm`: chain `base_link` → `link5`;
- group `gripper`: joint `gripper_joint`;
- named states `start`, `turn`, `open`, `closed`;
- `disable_collisions` for adjacent links and for the `gripper_finger` /
  `fixed_jaw` pair (the jaw is a single body in the URDF, so the model reports
  a permanent contact).

Change the chain ends if the tip or base link is renamed, edit the named states
to match the new start/open/closed angles, and regenerate the
`disable_collisions` list if links are added or removed. The usual way is to let
MoveIt Setup Assistant produce the SRDF; the file is plain XML and can be edited
by hand.

## 3. Joint limits

`config/joint_limits.yaml` holds `max_velocity` and `max_acceleration` in rad/s
and rad/s². The stock values are 1.0 and 3.0. They also define the 100 % speed
used by the web program editor and by MoveIt's scaling factor.

## 4. IK and planning

`config/kinematics.yaml` uses KDL with `position_only_ik: true`, which lets the
5-DOF arm reach XYZ targets without a full orientation. Keep it unless the new
arm has a different joint count or needs a different IK plugin.

`config/ompl_planning.yaml` contains the OMPL pipeline and request/response
adapters. It does not depend on the robot dimensions.

## 5. Controller

`config/moveit_controllers.yaml` declares one `FollowJointTrajectory`
controller named `arm_controller` and lists all six joints. If the new robot has
different joint names, change the list. The executor exposes the matching
action `/arm_controller/follow_joint_trajectory`.

## 6. Serial bridge

`minipulator/serial_bridge.py` contains the calibration:

```python
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_joint"]
SERVO_ZERO = [90.0, 90.0, 10.0, 90.0, 0.0, 0.0]
SERVO_SIGN = [1.0, -1.0, 1.0, -1.0, 1.0, 1.0]
SERVO_MIN  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SERVO_MAX  = [180.0, 180.0, 90.0, 180.0, 180.0, 90.0]
START      = [90.0, 90.0, 0.0, 40.0, 0.0, 0.0]
```

- `JOINTS` must be in the same order as the servos on the board.
- `SERVO_ZERO` is the servo angle at which the model angle is zero. It is a
  calibration value: set it by moving the joint to its model zero and reading
  the angle the board reports.
- `SERVO_SIGN` flips a joint whose positive direction is opposite to the model.
- `SERVO_MIN` / `SERVO_MAX` clamp the servo commands. They must match the
  firmware values.
- `START` is the parking pose in servo degrees. It is sent on boot and when the
  web UI's **Home** button is pressed. Keep it inside the mechanical range.

The bridge auto-detects the port: `/dev/minipulator` (udev), then a CH340
`ttyUSB*`. If the new board uses another USB chip, adjust `pick_port()` or pass
`--port` explicitly. `--baud` (default 115200), `--speed` (default 60 °/s) and
`--rate` (20 Hz) are command-line options.

## 7. Firmware

`firmware/minipulator/minipulator.ino`:

```cpp
const uint8_t AXES = 6;
const uint8_t PINS[AXES] = {4, 5, 6, 7, 8, 9};
const float SERVO_MIN[AXES] = {0, 0, 0, 0, 0, 0};
const float SERVO_MAX[AXES] = {180, 180, 90, 180, 180, 90};
const float START_ANGLE[AXES] = {90, 90, 0, 40, 0, 0};
const unsigned long BAUD = 115200;
const float MAX_SPEED = 150.0;
```

Change `PINS` to the pins the servos are connected to, `SERVO_MIN`/`SERVO_MAX`
to the safe mechanical range, and `START_ANGLE` to the parking pose. Keep
`BAUD` equal to the bridge's baud rate. `MAX_SPEED` caps the transition speed,
`WATCHDOG_MS` keeps the torque on when the PC stops sending commands, and
`REPORT_MS` is the state report period.

The protocol (`T`, `S`, `P`, `READY`, `PONG`, `E`) does not need to change for a
similar board. If you write your own firmware, keep those messages.

## 8. udev

`udev/99-minipulator.rules` matches the CH340 (`1a86:7523`) and creates
`/dev/minipulator`. Replace the vendor/product IDs with those of the new board
(`lsusb`) and the symlink name if you prefer another one. Then update the
default `port` in the launch files. Without a rule, pass `port:=/dev/ttyUSB0`.

## 9. Launch files

- `launch/moveit.launch.py`, `teleop.launch.py`, `web.launch.py`: default
  `port` is `/dev/minipulator`.
- `launch/teleop.launch.py`: `START_Q` must match the bridge `START` pose in
  radians. The sliders start there, otherwise the first tick would move the arm.
- `launch/web.launch.py`: `host` and `web_port` defaults.

All launch files read the URDF and the MoveIt YAML from the package share
directory, so no other paths need editing.

## 10. Web UI backend

`minipulator/web_ui.py`:

```python
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
TIP = "link5"
GRIP_OPEN = 1.2
GRIP_CLOSE = 0.3
MAX_STEP = 0.15
```

Update the joint names and tip link if renamed, the gripper open/close angles
for a different jaw, and `MAX_STEP` if the IK solutions jump branches for the new
arm. Programs and the library are stored under `~/.minipulator`, independent of
the robot.

## 11. Web UI frontend

`web/app.js`:

```js
const ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'];
const GRIP = 'gripper_joint';
const TIP = 'link5';
const LABEL = { joint1: 'j1', ... };
```

The 3D model is built from `urdf/robot.urdf` at runtime, and joint limits come
from the URDF, so only the names and labels need editing. The page title and
the axis legend are in `web/index.html`.

## 12. RViz

`rviz/moveit.rviz` and `rviz/teleop.rviz` store the fixed frame, the
MotionPlanning group and the robot description topic. If the group or the fixed
frame changes, open RViz, adjust the displays and save the configuration over
the file. A wrong group in RViz is the usual reason the MotionPlanning panel
shows no robot.

## 13. Demos and programs

`minipulator/demos.py` contains taught poses for this arm. Re-teach them with
the web UI or the sliders, then copy the joint values. Saved programs are JSON
files and can be edited by hand; they live outside the package.

## Verification

Work through this list after the changes:

1. `ros2 launch minipulator view.launch.py` — the model matches the real arm,
   every joint moves around the right axis.
2. `ros2 launch minipulator teleop.launch.py` — move one joint by a few degrees.
   If it moves the wrong way, flip the corresponding `SERVO_SIGN` value; if the
   zero is wrong, adjust `SERVO_ZERO`.
3. Command the zero pose (`q = 0`) and check the physical zero. Write down the
   servo angles; they are the `SERVO_ZERO` calibration.
4. Command the start pose; it must match `START` and `START_Q`.
5. Open the web UI: the wrist point in 3D should agree with the real arm.
   Drag the gizmo 2 cm; the arm should follow smoothly.
6. In RViz, plan a small joint goal and execute it. A failure here is usually
   the start state (bridge not running) or a collision in the URDF.
7. Move to the limits; the firmware and the bridge must clamp, not grind.
8. Teach two points in the web UI, run the program, press STOP mid-move.
9. Check the log tab for `MoveIt: ошибка` and `IK: решения нет`.

## Notes for similar hardware

- Servo pulse ranges differ between models. Set `SERVO_MIN`/`SERVO_MAX` from the
  mechanical range, not from the 0–180° nominal range.
- The arm is open loop. After a power cycle or a stall, the real pose may differ
  from `/joint_states`; re-home before running programs.
- The Nano's 5 V regulator cannot power servos. Use a separate supply with a
  common ground.
- If the new board is not a CH340, update the udev rule or the default port.
- If the new arm has a different joint count, the six-joint assumptions in the
  bridge, the controller, the SRDF and the web UI must be changed together.
