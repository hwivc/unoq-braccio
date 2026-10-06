# Arduino Braccio + ROS 2

A TinkerKit Braccio arm that finds coloured cubes with a camera (colour
detection + Edge Impulse), works out the joint angles with inverse
kinematics, and sorts each cube into the bin of its colour. It runs in Gazebo
simulation, and on the real arm with an **Arduino UNO over USB serial**.

<p align="center">
  <img src="docs/braccio.gif" width="800" alt="Braccio demo">
</p>

| Gazebo simulation | Real Braccio |
|---|---|
| <img width="400" alt="Gazebo simulation" src="https://github.com/user-attachments/assets/7736b383-4374-40f8-bcf9-347998eaff60" /> | <img width="400" alt="Braccio on the bench" src="https://github.com/user-attachments/assets/db8d983e-142a-4fbe-8bbe-c46fc83fc3e3" /> |

**Contents:** [Build](#build) · [Simulation](#simulation) ·
[Real arm](#real-arm-arduino-uno) · [Move the arm](#move-the-arm) ·
[Vision](#vision-and-edge-impulse) · [Troubleshooting](#troubleshooting) ·
[Layout](#repository-layout) · [Docs](#more-documentation)

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
Linux PC / Raspberry Pi 5 (ROS 2, cameras)  --USB-->  Arduino UNO + Braccio shield  -->  servos
```

1. **Flash the UNO** (Arduino UNO R3; R4 Minima/WiFi also work):

   ```bash
   bash scripts/flash_uno.sh /dev/ttyACM0            # Linux
   ```
   ```powershell
   .\scripts\flash_uno.ps1 -Port COM3           # Windows
   ```

2. **One-time Linux setup** (fixed port name `/dev/braccio` + serial permission):

   ```bash
   bash scripts/setup_uno_serial.sh                  # then log out and back in
   ```

3. **Start the bridge:**

   ```bash
   ros2 launch unoq_braccio_bringup hardware.launch.py serial_port:=/dev/braccio
   ros2 launch unoq_braccio_bringup hardware.launch.py serial_port:=/dev/braccio speed:=40 rviz:=true
   ```

Power the servos from the shield's own 5 V supply (4 A or more), never from
USB. On power-up the arm stands straight up, and RViz shows it upright from the
start; the arm then waits about 8 s while the UNO resets and gently powers
up its servos. Wiring, serial protocol and troubleshooting:
[docs/hardware.md](docs/hardware.md).

### Real pick and place with a camera

The overhead camera can be a USB webcam (`camera:=0`) or a WiFi stream, e.g.
a phone running the IP Webcam app (`camera:=http://<phone-ip>:8080/video`).

1. Mount the camera **looking straight down**, top of the picture pointing the
   way the arm faces.
2. Start the arm and camera (this includes everything `hardware.launch.py`
   starts; never run both):

   ```bash
   ros2 launch unoq_braccio_bringup real.launch.py serial_port:=/dev/braccio \
     camera:=http://192.168.1.192:8080/video
   ```

3. In a second terminal, run the step-by-step set-up:

   ```bash
   ros2 run unoq_braccio_driver real_setup
   ```

   | Step | You do |
   |---|---|
   | 1 Camera direction | `+` / `-` turn the arm until it points at the spot under the camera, Enter |
   | 2 Measurements | Type distance base-to-that-spot, camera height, cube size (mm) |
   | 3 Colours | One cube at a time under the camera: `s` saves the colour shown, `n` done |
   | 4 Drop points | Per colour: `+` / `-` turn, `w` / `s` further / nearer, Enter saves |

   It saves everything to `~/.ros/braccio_setup.yaml`. Restart step 2 afterwards.

4. Sort the cubes:

   ```bash
   ros2 launch unoq_braccio_bringup real_pick_place.launch.py
   ```

No gripper camera is needed (`gripper_camera:=false` is the default). Full
guide: [docs/camera.md](docs/camera.md).

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

The cube detector uses an Edge Impulse model to find the cubes (colour then
decides which cube is which). The model (~156 MB) is too big for GitHub, so
download it once; without it the detector falls back to plain colour
detection and logs a warning.

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
| Real camera: where does it see the cubes? | `ros2 topic echo /vision/cube_target` |
| Which serial port? | `ls /dev/ttyACM* /dev/ttyUSB* /dev/braccio` |
| Layout / IK / protocol / calibration checks (no ROS needed) | `python ros2_ws/src/unoq_braccio_driver/test/test_workspace.py` (also `test_protocol.py`, `test_camera_calibration.py`) |

## Repository layout

```text
firmware/braccio_uno_firmware/   Arduino UNO serial firmware (servos only)
ros2_ws/src/unoq_braccio_driver/ ROS 2 nodes: serial bridge, kinematics, vision, pick and place
ros2_ws/src/unoq_braccio_sim/    URDF, Gazebo world, controllers
ros2_ws/src/unoq_braccio_bringup/ Launch files
scripts/                         Flashing and setup scripts
docs/                            Hardware, vision, roadmap and other guides
edge_impulse/, test/             Edge Impulse integration and the cube model test tool
app_lab/, web_app/               Arduino UNO Q apps and browser dashboard (alternative set-up)
```

## More documentation

| Topic | Where |
|---|---|
| Real arm: wiring, protocol, troubleshooting | [docs/hardware.md](docs/hardware.md) |
| Real camera (USB / phone IP Webcam) and calibration | [docs/camera.md](docs/camera.md) |
| Manual control and gripper calibration | [docs/manual_control.md](docs/manual_control.md) |
| Simulation internals | [ros2_ws/src/unoq_braccio_sim/README.md](ros2_ws/src/unoq_braccio_sim/README.md) |
| Install on Windows / macOS / Linux | [docs/platform-setup.md](docs/platform-setup.md) |
| Camera vision | [docs/vision.md](docs/vision.md) |
| Edge Impulse integration, data capture | [edge_impulse/README.md](edge_impulse/README.md), [edge_impulse/data_capture.md](edge_impulse/data_capture.md) |
| Roadmap: tutorial plan, new sim features | [docs/roadmap.md](docs/roadmap.md) |
| Arduino UNO Q set-up (network, App Lab, web dashboard) | [docs/architecture.md](docs/architecture.md), [web_app/README.md](web_app/README.md) |
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
