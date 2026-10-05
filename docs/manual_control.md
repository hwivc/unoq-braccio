# Manual Control and Calibration

The `manual_control` node allows real-time interactive arm jogging and gripper calibration for both Gazebo simulation and the physical robot arm. It supports both **USB joysticks/gamepads** (such as generic Ucom/Microtik controllers and Xbox gamepads) and **keyboard teleoperation**.

## Launching Manual Control

**Option 1: USB Joystick / Gamepad (Ucom / Microtik / Generic default)**
```bash
# Default (first joystick /dev/input/js0):
ros2 launch unoq_braccio_bringup manual_control.launch.py input_type:=joystick controller_type:=ucom

# Specify a specific USB device (e.g., js1, /dev/input/js1, or 1):
ros2 launch unoq_braccio_bringup manual_control.launch.py device_id:=1

# For an Xbox gamepad layout:
ros2 launch unoq_braccio_bringup manual_control.launch.py input_type:=joystick controller_type:=xbox
```

> **Note for Ucom / Microtik Gamepads:** Press the **ANALOG** button on your gamepad (ensure the red LED is ON) so that the analog thumbsticks are activated.

**Option 2: Keyboard Teleoperation**
```bash
# Keyboard mode needs an interactive terminal, so use ros2 run:
ros2 run unoq_braccio_driver manual_control --ros-args -p input_type:=keyboard
```

**Option 3: Joint State Publisher GUI (Visual Sliders Window)**
```bash
# Launch the visual sliders window (forwarding movements directly to the arm):
ros2 launch unoq_braccio_bringup joint_state_publisher_gui.launch.py

# If you also want a standalone RViz window:
ros2 launch unoq_braccio_bringup joint_state_publisher_gui.launch.py rviz:=true
```

## Controls & Gripper Calibration

There are two modes, toggled with `Select` / `Back` / `M`:

- **TOOL** (default): move the fingertip. The base turns, the tip reaches
  out/in and up/down, and the tool keeps its pitch while you move. If a move
  would leave the workspace, the pitch gives way by up to 30 degrees, then
  the blocked direction stops while the others keep working.
- **JOINT**: drive each servo directly.

| Function | Ucom / Microtik Gamepad | Xbox Gamepad | Keyboard |
| :--- | :--- | :--- | :--- |
| **Turn base** (TOOL) / base (JOINT) | Left stick X | Left stick X | `A` / `D` |
| **Reach out / in** (TOOL) / shoulder (JOINT) | Left stick Y | Left stick Y | `W` / `S` |
| **Up / down** (TOOL) / elbow (JOINT) | Right stick Y | Right stick Y | `R` / `F` |
| **Tilt tool** (TOOL) / wrist_vertical (JOINT) | D-pad up / down | D-pad up / down | `T` / `G` |
| **Roll wrist** | Right stick X | Right stick X | `Q` / `E` |
| **Gripper open / close** (hold) | `L1` / `R1` | `LB` / `RB` | `[` / `]` |
| **Snap Gripper Open** (10°) | `Cross` (3) | `A` | `O` |
| **Snap Gripper Closed** (95°) | `Circle` (2) | `B` | `C` |
| **Go to Ready Pose** | `Square` (4) | `X` | `H` |
| **Print Calibration Report** | `Triangle` (1) | `Y` | `Y` |
| **Switch Mode (TOOL <-> JOINT)** | `Select` | `Back / View` | `M` |
| **Toggle Speed (Normal <-> Precision)** | `Start` | `Start / Menu` | `P` |

Stick response is gentle near the centre and full speed at the edge, and the
output is slew-limited (`max_joint_rate`, default 120 deg/s), so the arm never
jerks. If a stick or button is on a different index on your pad, check with
`ros2 topic echo /joy` and override it, for example
`--ros-args -p axis_rx:=2 -p button_mode:=8`. Speeds are parameters too
(`linear_speed`, `yaw_speed`, `pitch_speed`, `joint_speed`, `gripper_speed`,
`precision_scale`).

## Gripper Calibration Workflow

1. Start the simulation (`ros2 launch unoq_braccio_bringup sim.launch.py`) or hardware bridge.
2. Launch `manual_control`.
3. Jog the arm over a cube and descend until the fingers surround the cube center.
4. Hold the gripper close button (`R1`/`RB`, or tap `]`) until the cube is firmly gripped. Press `Start`/`P` first for slow, fine steps.
5. Press **`Y`** (or Triangle on Ucom). The terminal prints a formatted calibration report with the exact values to copy & paste into `braccio_kinematics.py` (`GRIPPER_CLOSED = ...`) and `braccio_workspace.py` (`CUBE_CENTRE_Z = ...`).
