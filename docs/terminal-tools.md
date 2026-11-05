# Terminal tools

The web page does all of this; these are for SSH, scripting or debugging. They use the same sessions (`~/.ros/braccio_teach/<session>/`) and settings files as the web page.

## Arm only (check it moves, no camera)

```bash
ros2 launch unoq_braccio_bringup hardware.launch.py serial_port:=auto     # or /dev/braccio, /dev/ttyACM0
```

The arm stands up and waits about 8 s while the UNO resets. Wiring and protocol: [docs/hardware.md](hardware.md).

## Teach in the terminal

**Start the arm and camera** (terminal 1; it includes `hardware.launch.py`,
never run both; `web:=true` adds the web page):

```bash
ros2 launch unoq_braccio_bringup real.launch.py serial_port:=/dev/braccio \
  camera:=http://192.168.1.192:8080/video
```

**Cube colours** (terminal 2). Put one cube under the camera,
`s` saves its colour, next cube, `n` when done. Run it again any time to add a
colour; the ones already saved are kept (`x` forgets them).

```bash
ros2 run unoq_braccio_driver real_setup --ros-args -p steps:=colors
```

Restart terminal 1 after adding colours.

**Record examples** (terminal 2):

```bash
ros2 run unoq_braccio_driver teach_pick --ros-args -p session:=new
```

For each example, put ONE cube down and press Enter. The camera reads it, then:

| Stage | You do |
|---|---|
| ABOVE | Drive the open gripper above the cube, Enter |
| GRAB | Lower the open fingers around it, Enter |
| CLOSE | Close (`C`), lift (e.g. `k`), Enter |
| CHECK | `y` if it holds the cube (saved), `n` if not (not saved) |

Keys: `j`/`l` base, `i`/`k` shoulder, `y`/`h` elbow, `t`/`g` wrist, `r`/`f`
roll, `o`/`c` gripper (`O` open, `C` grip), Tab switches 1/5 degree steps,
Esc cancels the example. From the second example on, the arm starts at its own
guess, so you only correct it.

Menu keys: Enter record · `T` test · `D` drop poses · `R` review · `U` undo
last example · `Q` quit. Everything is saved in
`~/.ros/braccio_teach/<session>/`; run the same session again to continue.

**Review:** press `R` in `teach_pick` (or, without the arm,
`python3 -m unoq_braccio_driver.pick_learning desk` from
`ros2_ws/src/unoq_braccio_driver`). It lists examples whose angles disagree
with their neighbours or whose camera reading wobbled, offers to delete them,
and shows where tests missed so you know where to teach more.

**Drop poses:** `D` in `teach_pick` teaches one per colour; without them: then cubes are dropped at the far end of the base
rotation (`l` key direction), reaching out as far as they were picked.

## Sort in the terminal

**Sort** (terminal 2, with terminal 1 still running):

```bash
# every colour, until no cube it can pick is left
ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=desk

# one colour, leave the others
ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=desk colors:=red

# several colours, always drop at the side (ignore taught drop poses)
ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=desk colors:=red,blue drop:=side
```

Options: `drop:=auto` (taught pose if there is one, else side; default),
`drop:=taught` (only colours with a taught pose), `drop:=side`;
`drop_base:=0` the base angle for side drops; `model:=model_0010` an older
model; `unsure_px:=60` how far from examples it still tries. Each pick is
checked with the camera; a cube is tried twice before it is left. Ctrl+C
freezes the arm. Watch it with `ros2 topic echo /task/state`.

## Live demo: copy and paste

Terminal 1, the arm and camera (keep it running; `web:=true` also starts the
[web dashboard](web-dashboard.md)):

```bash
ros2 launch unoq_braccio_bringup real.launch.py serial_port:=/dev/braccio \
  camera:=http://192.168.1.192:8080/video web:=true
```

Terminal 2. Add a colour first if it is new (one cube under the camera, `s`,
then `n`; restart terminal 1 afterwards):

```bash
ros2 run unoq_braccio_driver real_setup --ros-args -p steps:=colors
```

Pick only one colour (here red) out of the others, dropping at the far end of
the `l` key:

```bash
ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=new colors:=red drop:=side
```

Pick every colour, continuously until none are left, all dropped at the `l` end:

```bash
ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=new drop:=side
```

Use your session name (`new` here) and the colour names `real_setup` printed.
Only cubes within 60 px of taught examples are picked, and the side drops all
land in one spot, so keep it clear.


## Other launch files

| Launch | What |
|---|---|
| `pi.launch.py` | **The one to use**: real arm + camera + web page ([Run it](https://github.com/hwivc/unoq-braccio/blob/main/README.md#quick-start)) |
| `real.launch.py` | The same pieces with every option (`camera:=`, `serial_port:=`, `web:=true`, `rviz:=`, `detector_backend:=` …); `pi.launch.py` is this with good defaults |
| `hardware.launch.py` | Arm only (serial bridge + 3D model) |
| `sim.launch.py` | Gazebo simulation (`web:=true` for the web page) |
| `learned_pick_place.launch.py` | Sort with a taught model, from a terminal |
| `manual_control.launch.py`, `joint_state_publisher_gui.launch.py` | Drive the joints by keyboard, joystick or sliders |
| `real_pick_place.launch.py`, `vision_*.launch.py`, `edge_impulse_*.launch.py`, `data_capture.launch.py`, `remote.launch.py` | Older set-ups: measured camera + IK, Edge Impulse, the UNO Q |

## Move the arm

These work the same in simulation and on the real arm.

```bash
ros2 run unoq_braccio_driver pose_demo --ros-args -p pose:=ready    # rest, ready, grip_test, grip_full, pickup, drop, wave
ros2 run unoq_braccio_driver ik_pose_demo --ros-args -p x:=0.20 -p y:=0.0 -p z:=0.10 -p gripper:=25   # metres
ros2 run unoq_braccio_driver manual_control --ros-args -p input_type:=keyboard
ros2 launch unoq_braccio_bringup manual_control.launch.py input_type:=joystick controller_type:=xbox
ros2 launch unoq_braccio_bringup joint_state_publisher_gui.launch.py          # slider window
```

Send raw servo angles (degrees: base, shoulder, elbow, wrist_vertical, wrist_rotation, gripper):

```bash
ros2 topic pub --once /braccio/joint_command sensor_msgs/msg/JointState \
  "{name: [base, shoulder, elbow, wrist_vertical, wrist_rotation, gripper], position: [90, 90, 90, 90, 90, 25]}"
```

Controls and gripper calibration: [docs/manual_control.md](manual_control.md).

## Troubleshooting

| Problem | Command |
|---|---|
| Sim arm does not move | `ros2 topic hz /clock` (must tick) and `ros2 control list_controllers` |
| Is anything commanding the arm? | `ros2 topic echo /braccio/joint_command` |
| Real arm status | `ros2 topic echo /braccio/firmware_status` |
| Real camera: which cubes does it see? | open `teach_pick`: its menu line shows the cubes it sees right now |
| Learned picks miss | `R` in `teach_pick`, then teach where it misses |
| Which serial port? | `ls /dev/ttyACM* /dev/ttyUSB* /dev/braccio`, or Settings → Devices on the web page |
| "No arm found" / "No Grove camera found" | The board is not seen by Linux at all: `lsusb` must list `2341:0043` (UNO) and `1a86:55d3` (Grove). Re-plug the cable or hub; use a powered hub or plug straight in (`journalctl -k \| grep usb` shows drops) |
| Camera sees no cubes | No colours saved, or they were measured with another camera or light: Sort → Add colour, tap each cube again. 💧 on the camera shows what each colour matches; dashed boxes say why a blob was rejected (e.g. held too close: too big) |
| Web page shows old features | Reload it once after an update |
| Layout / IK / protocol / calibration checks (no ROS needed) | `python ros2_ws/src/unoq_braccio_driver/test/test_workspace.py` (also `test_protocol.py`, `test_camera_calibration.py`) |

## Arm tricks

With the arm running (`hardware.launch.py`, `real.launch.py` or `sim.launch.py`),
in a second terminal. Each one ends standing up straight; add
`-p speed:=0.5` for slower or `-p speed:=1.5` for faster.

```bash
ros2 run unoq_braccio_driver arm_tricks --ros-args -p trick:=wave    # waves hello
ros2 run unoq_braccio_driver arm_tricks --ros-args -p trick:=dance   # sways side to side
ros2 run unoq_braccio_driver arm_tricks --ros-args -p trick:=nod     # nods "yes"
ros2 run unoq_braccio_driver arm_tricks --ros-args -p trick:=shake   # shakes "no"
ros2 run unoq_braccio_driver arm_tricks --ros-args -p trick:=bow     # bows, then snaps the gripper
```
