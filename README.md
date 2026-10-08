# Arduino Braccio + ROS 2

A TinkerKit Braccio arm that finds coloured cubes with an overhead camera and
sorts them. On the **real arm** (Arduino UNO over USB serial) you teach it to
pick by driving it onto cubes a few dozen times; a small learned model (kNN)
then maps what the camera sees straight to servo angles, so no camera or arm
calibration is needed. In **Gazebo simulation** it uses inverse kinematics.

<p align="center">
  <img src="docs/braccio.gif" width="800" alt="Braccio demo">
</p>

| Gazebo simulation | Real Braccio |
|---|---|
| <img width="400" alt="Gazebo simulation" src="https://github.com/user-attachments/assets/7736b383-4374-40f8-bcf9-347998eaff60" /> | <img width="400" alt="Braccio on the bench" src="https://github.com/user-attachments/assets/db8d983e-142a-4fbe-8bbe-c46fc83fc3e3" /> |

**Contents:** [Build](#build) · [Simulation](#simulation) ·
[Real arm](#real-arm-arduino-uno) · [Teach it to pick](#teach-the-real-arm-to-pick) ·
[Web dashboard](#web-dashboard) · [Move the arm](#move-the-arm) · [Vision](#vision-and-edge-impulse) ·
[Troubleshooting](#troubleshooting) · [Layout](#repository-layout) ·
[Docs](#more-documentation) · [What next](docs/learning_roadmap.md)

## Build

Ubuntu 24.04 + ROS 2 Jazzy. Full install steps for each OS:
[docs/platform-setup.md](docs/platform-setup.md).

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-xacro
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run `source ros2_ws/install/setup.bash` in every new terminal.

## Simulation

```bash
ros2 launch unoq_braccio_bringup sim.launch.py                 # Gazebo + RViz + cameras
```

In a second terminal:

```bash
ros2 run unoq_braccio_driver pick_place_demo                   # sort every cube into its bin
ros2 run unoq_braccio_driver pick_place_demo --ros-args -p colors:="[blue]"   # one colour only
ros2 topic echo /task/state                                    # watch the state machine
```

The pick-and-place goes through these states:
`DETECTING → MOVE_ABOVE_CUBE → DESCEND → GRASP → LIFT → MOVE_TO_BIN → RELEASE → VERIFY_PLACEMENT → COMPLETE`.
Red cubes go to the green bin, blue to cyan, yellow to magenta.

Launch options:

```bash
ros2 launch unoq_braccio_bringup sim.launch.py rviz:=false                  # Gazebo only
ros2 launch unoq_braccio_bringup sim.launch.py detector_backend:=color_blob # no ML model
ros2 launch unoq_braccio_bringup sim.launch.py model_path:=/path/model.lite # your own model
```

How the simulation works (cameras, detection tuning, grasp assist, servo
conventions): [ros2_ws/src/unoq_braccio_sim/README.md](ros2_ws/src/unoq_braccio_sim/README.md).

## Real arm (Arduino UNO)

```text
PC / Raspberry Pi 5 (ROS 2, camera)  --USB-->  Arduino UNO + Braccio shield  -->  6 servos
```

**Hardware:** TinkerKit Braccio with its shield on an Arduino UNO R3 (R4
Minima/WiFi also work), a 5 V supply of 4 A or more for the servos, and an
overhead camera: a USB webcam (`camera:=0`) or a phone running the IP Webcam
app (`camera:=http://<phone-ip>:8080/video`). Power the servos from the
shield's own supply, never from USB.

1. **Flash the UNO** (once):

   ```bash
   bash scripts/flash_uno.sh /dev/ttyACM0            # Linux
   ```
   ```powershell
   .\scripts\flash_uno.ps1 -Port COM3                # Windows
   ```

2. **Serial port setup** (once on Linux: fixed name `/dev/braccio` + permission):

   ```bash
   bash scripts/setup_uno_serial.sh                  # then log out and back in
   ```

3. **Check the arm moves** (bridge only, no camera):

   ```bash
   ros2 launch unoq_braccio_bringup hardware.launch.py serial_port:=/dev/braccio
   ```

   On power-up the arm stands straight up and waits about 8 s while the UNO
   resets and powers its servos gently. Wiring, serial protocol and
   troubleshooting: [docs/hardware.md](docs/hardware.md).

## Teach the real arm to pick

Each example stores what the camera saw (cube position and angle) and the
servo angles you drove the arm to. The model blends the nearest examples
(kNN), so servo offsets, bent links and a tilted camera do not matter: it
repeats what the real arm did. Every 5 examples it also tries polynomial and
small neural-network models and keeps whichever predicts best.

**1. Mount the camera** looking down at the table, so it sees the whole area
where cubes go. Do not move or zoom it afterwards (if you do, start a new
session).

**2. Start the arm and camera** (terminal 1; it includes `hardware.launch.py`,
never run both):

```bash
ros2 launch unoq_braccio_bringup real.launch.py serial_port:=/dev/braccio \
  camera:=http://192.168.1.192:8080/video
```

**3. Teach it your cube colours** (terminal 2). Put one cube under the camera,
`s` saves its colour, next cube, `n` when done. Run it again any time to add a
colour; the ones already saved are kept (`x` forgets them).

```bash
ros2 run unoq_braccio_driver real_setup --ros-args -p steps:=colors
```

Restart terminal 1 after adding colours.

**4. Record examples** (terminal 2):

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

Tips that matter most:

- **Spread the examples** over the whole area, about every 3-4 cm, with the
  cube at different angles. 25-40 examples cover a desk-sized area.
- **Reach each spot the same way.** Keep a similar posture (e.g. elbow high,
  wrist tipped down) in neighbouring examples; two different postures for
  the same spot get averaged into one that misses.
- Every 5 examples it offers a **test**: put a cube anywhere, the arm picks on
  its own (`s` stops it). If it misses, answer `y` to "Correct it?" and drive
  it onto the cube: that becomes an example exactly where it was weakest.
- Cubes more than 60 px from every example are refused, not guessed.

Menu keys: Enter record · `T` test · `D` drop poses · `R` review · `U` undo
last example · `Q` quit. Everything is saved in
`~/.ros/braccio_teach/<session>/`; run the same session again to continue.

**5. Review and clean up.** Press `R` in `teach_pick` (or, without the arm,
`python3 -m unoq_braccio_driver.pick_learning desk` from
`ros2_ws/src/unoq_braccio_driver`). It lists examples whose angles disagree
with their neighbours or whose camera reading wobbled, offers to delete them,
and shows where tests missed so you know where to teach more.

**6. Where cubes go.** Either teach a drop pose per colour (`D` in
`teach_pick`), or skip it: then cubes are dropped at the far end of the base
rotation (`l` key direction), reaching out as far as they were picked.

**7. Run it** (terminal 2, with terminal 1 still running):

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

### Live demo: copy and paste

Terminal 1, the arm and camera (keep it running; `web:=true` also starts the
[web dashboard](#web-dashboard)):

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

Before kNN, the real arm used a measured camera and inverse kinematics
(`real_setup` with all steps, then `real_pick_place.launch.py`). That still
works and is described in [docs/camera.md](docs/camera.md), but it needs
careful calibration and missed more often.

## Web dashboard

Watch and control the arm from any phone, tablet or computer on the same
network: a live 3D model of the arm (the same URDF and transforms RViz uses),
the overhead camera, what the arm is doing, and the controls. It works with
the real arm and with the simulation.

```bash
ros2 launch unoq_braccio_bringup real.launch.py ... web:=true   # with the real arm
ros2 launch unoq_braccio_bringup sim.launch.py web:=true        # with Gazebo
ros2 run unoq_braccio_driver web_dashboard                      # or on its own, next to either
```

The terminal prints the address (e.g. `http://192.168.1.31:8000`) and, the
first time, a **pairing code**: open the address, enter the code and choose a
name and password. That makes you the owner. Invite others in Settings →
**Show Pairing Code**: each code works once, for 10 minutes, as an
**operator** (can control), **viewer** (can only watch) or **owner** (can also
invite people and see Advanced).

| Where | What |
|---|---|
| Sort | Pick colours (or All), drop at their spot or to the side, Start. Shows each step live and the cubes placed |
| Move | Gripper open / close, Home and other poses; Joints (folded away) has a slider per servo |
| Stop (red, bottom right) | Stops the task and holds the arm where it is. Always there; `Esc` on a keyboard |
| Camera (top right) | Tap to swap with the 3D view |
| Settings | Account, theme, Reduce Transparency, people; Advanced: teaching session, model, drop mode, examples review, activity log, raw servo values |

Only one person drives at a time; others see who is in control and can take
over. Drag to orbit the 3D view, pinch or scroll to zoom, double-click to reset.
Accounts are in `~/.ros/braccio_web.yaml` (salted password hashes; delete an
entry to remove someone). Options: `-p port:=8000`, and
`-p certfile:=... -p keyfile:=...` for HTTPS if the network is not trusted.

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

Controls and gripper calibration: [docs/manual_control.md](docs/manual_control.md).

## Vision and Edge Impulse

In simulation the cube detector uses an Edge Impulse model to find the cubes
(colour then decides which cube is which). On the real arm the default is
plain colour detection with the colours learned by `real_setup`: it is steadier
than the 64x64 model on a real camera. Use the model there with
`real.launch.py detector_backend:=edge_impulse`. The model (~156 MB) is too
big for GitHub, so download it once; without it the detector falls back to
plain colour detection and logs a warning.

```bash
ros2 run unoq_braccio_driver download_model                  # saves it to ~/unoq-braccio/ (resumable, Ctrl+C safe)
pip install --break-system-packages ai-edge-litert 'numpy<2' # TFLite runtime, once; numpy<2 keeps Ubuntu's OpenCV working
```

Restart the launch afterwards; it should log `Cube detector: Edge Impulse model '...'`.

- Cube detection project (this model): <https://studio.edgeimpulse.com/studio/975321>
- Direct model link (float32): <https://studio.edgeimpulse.com/v1/api/975321/learn-data/3/model/tflite-float>

```bash
ros2 launch unoq_braccio_bringup vision_usb.launch.py camera_index:=0 label:=object    # USB camera on the ROS machine
cd test && python detect.py                                                           # try the cube model on images, no ROS
```

Public Edge Impulse project: <https://studio.edgeimpulse.com/studio/1029890>.
Keep API keys in `EDGE_IMPULSE_API_KEY`, never in committed files. Details:
[docs/vision.md](docs/vision.md), [edge_impulse/README.md](edge_impulse/README.md),
[test/README.md](test/README.md).

## Troubleshooting

| Problem | Command |
|---|---|
| Sim arm does not move | `ros2 topic hz /clock` (must tick) and `ros2 control list_controllers` |
| Is anything commanding the arm? | `ros2 topic echo /braccio/joint_command` |
| Real arm status | `ros2 topic echo /braccio/firmware_status` |
| Real camera: which cubes does it see? | open `teach_pick`: its menu line shows the cubes it sees right now |
| Learned picks miss | `R` in `teach_pick`, then teach where it misses |
| Which serial port? | `ls /dev/ttyACM* /dev/ttyUSB* /dev/braccio` |
| Layout / IK / protocol / calibration checks (no ROS needed) | `python ros2_ws/src/unoq_braccio_driver/test/test_workspace.py` (also `test_protocol.py`, `test_camera_calibration.py`) |

## Repository layout

```text
firmware/braccio_uno_firmware/   Arduino UNO serial firmware (servos only)
ros2_ws/src/unoq_braccio_driver/ ROS 2 nodes: serial bridge, vision, teaching (teach_pick, pick_learning), pick and place
ros2_ws/src/unoq_braccio_sim/    URDF, Gazebo world, controllers
ros2_ws/src/unoq_braccio_bringup/ Launch files
scripts/                         Flashing and setup scripts
docs/                            Hardware, vision, roadmap and other guides
edge_impulse/, test/             Edge Impulse integration and the cube model test tool
ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/web/  Web dashboard page (three.js vendored, works offline)
legacy/uno_q/                    Earlier Arduino UNO Q set-up: App Lab apps, old web app, UNO Q firmware
```

## More documentation

| Topic | Where |
|---|---|
| Real arm: wiring, protocol, troubleshooting | [docs/hardware.md](docs/hardware.md) |
| Real camera (USB / phone IP Webcam) and calibration | [docs/camera.md](docs/camera.md) |
| What to build next: stacking, building, models | [docs/learning_roadmap.md](docs/learning_roadmap.md) |
| Manual control and gripper calibration | [docs/manual_control.md](docs/manual_control.md) |
| Simulation internals | [ros2_ws/src/unoq_braccio_sim/README.md](ros2_ws/src/unoq_braccio_sim/README.md) |
| Install on Windows / macOS / Linux | [docs/platform-setup.md](docs/platform-setup.md) |
| Camera vision | [docs/vision.md](docs/vision.md) |
| Edge Impulse integration, data capture | [edge_impulse/README.md](edge_impulse/README.md), [edge_impulse/data_capture.md](edge_impulse/data_capture.md) |
| Roadmap: tutorial plan, new sim features | [docs/roadmap.md](docs/roadmap.md) |
| Earlier Arduino UNO Q set-up (App Lab, old web app) | [legacy/uno_q/README.md](legacy/uno_q/README.md) |
| Two-arm system plan | [docs/two_arm_braccio_system_architecture.md](docs/two_arm_braccio_system_architecture.md) |

## Credits

Built on ROS 2 and Gazebo (Open Robotics), Edge Impulse and Arduino. Arm
meshes and much inspiration from Will Stedden's
[braccio_moveit_gazebo](https://github.com/lots-of-things/braccio_moveit_gazebo)
(GPL-3.0).

## Fun: test the arm

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
