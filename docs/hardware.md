# Hardware: Arduino UNO + Braccio over USB serial

```text
Linux PC or Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy, cameras)
  -> USB cable -> Arduino UNO + Braccio shield -> 6 servos
```

The UNO only drives the servos. ROS 2, cameras and inverse kinematics run on
the Linux machine.

## Parts

- Arduino UNO R3 (or a UNO R4 Minima / WiFi)
- TinkerKit Braccio arm and Braccio shield
- 5 V power supply for the shield, 4 A or more
- USB cable to the Linux machine

## Wiring

Plug the Braccio shield onto the UNO and the servo cables into the shield:

| Shield | Joint | UNO pin |
|---|---|---:|
| M1 | base | 11 |
| M2 | shoulder | 10 |
| M3 | elbow | 9 |
| M4 | wrist vertical | 6 |
| M5 | wrist rotation | 5 |
| M6 | gripper | 3 |
| soft start | servo power ramp | 12 |

Power the servos from the shield's 5 V input. **Never power the servos from
USB, the UNO's 5 V pin or the Raspberry Pi.**

## 1. Flash the firmware

```bash
bash scripts/flash_uno.sh                         # UNO R3 on /dev/ttyACM0
bash scripts/flash_uno.sh /dev/ttyUSB0            # clone board (CH340 chip)
bash scripts/flash_uno.sh /dev/ttyACM0 arduino:renesas_uno:minima   # UNO R4 Minima
```

```powershell
.\scripts\flash_uno.ps1 -Port COM3           # Windows
```

You can also open `firmware/braccio_uno_firmware/braccio_uno_firmware.ino` in
the Arduino IDE (install the **Servo** library) and upload it.

On power-up the arm **stands straight up** (`ready`: 90 90 90 90 90, gripper
25), clear of the table, while the servo power ramps up gently over about 6
seconds. The UNO then reports that pose to ROS, and RViz shows the arm upright
from the moment it starts.

For a calm start, leave the arm roughly upright before switching the servo
power on and keep the space around it clear: the servos move to the start
pose as soon as they get power.

## 2. One-time Linux setup

```bash
bash scripts/setup_uno_serial.sh
```

This gives the board a fixed name, `/dev/braccio`, and adds you to the
`dialout` group (log out and back in afterwards).

## 3. Run

```bash
ros2 launch unoq_braccio_bringup hardware.launch.py serial_port:=/dev/braccio
ros2 launch unoq_braccio_bringup hardware.launch.py serial_port:=/dev/braccio speed:=40 rviz:=true
```

Then in another terminal:

```bash
ros2 run unoq_braccio_driver pose_demo --ros-args -p pose:=ready
ros2 run unoq_braccio_driver ik_pose_demo --ros-args -p x:=0.20 -p y:=0.0 -p z:=0.10
ros2 run unoq_braccio_driver manual_control --ros-args -p input_type:=keyboard
ros2 topic echo /braccio/firmware_status
```

For camera-driven pick and place on the real arm (USB webcam or a phone over
WiFi) see [camera.md](camera.md).

## Test the board without ROS

Open the Arduino Serial Monitor at **115200 baud**, line ending **Newline**,
and type:

```text
I                          -> READY BRACCIO_UNO 1
S                          -> STAT pos=... target=... moving=0 speed=60 ...
M 90 90 90 90 90 25        -> OK ... DONE      (arm stands up)
V 30                       -> OK               (slower)
H                          -> OK               (stop where it is)
```

## Serial protocol

One text line per command, 115200 baud:

| Command | Meaning | Reply |
|---|---|---|
| `M b s e wv wr g` | Move to servo angles (integer degrees, clamped to limits) | `OK` now, `DONE` on arrival |
| `S` | Status | `STAT pos=.. target=.. moving=0/1 speed=.. moves=.. uptime_ms=..` |
| `V n` | Speed of the furthest-moving joint, 10-180 deg/s | `OK` |
| `H` | Hold: stop where the arm is | `OK` |
| `I` | Identify | `READY BRACCIO_UNO 1` |

Anything else replies `ERR <reason>`. Motion is non-blocking: all joints
arrive together, and a new `M` during a move retargets smoothly.

The ROS `serial_bridge`:

- restarts the UNO when it connects (pulses the reset line), so the arm always
  begins standing up, then waits for it to boot before sending;
- sends only the newest command when they arrive faster than it can send;
- keeps the last angle for any joint a command leaves out;
- reconnects if the cable is unplugged;
- publishes the arm's real position on `/joint_states`.

## Joint limits

The firmware and ROS both clamp to these (`braccio_model.py`; a test checks
the firmware matches):

| Joint | Min | Max | Start pose (standing up) |
|---|---:|---:|---:|
| base | 0 | 180 | 90 |
| shoulder | 15 | 165 | 90 |
| elbow | 0 | 180 | 90 |
| wrist_vertical | 0 | 180 | 90 |
| wrist_rotation | 0 | 180 | 90 |
| gripper | 10 | 110 | 25 |

The stock Braccio library caps the gripper at 73, which does not close this
gripper, so the firmware drives the servos directly with `Servo.h`. Start
gripping at `95` and go towards `110` only if needed.

## Troubleshooting

| Problem | Check |
|---|---|
| `Lost /dev/braccio: device reports readiness to read but returned no data`, repeating | Run `bash scripts/check_uno_serial.sh` (with ROS stopped). Either another program is reading the port (ModemManager, Arduino IDE Serial Monitor, a second bridge), or the board browns out when the servos power up (servos on USB power / weak supply) |
| `Cannot open /dev/...` | `ls /dev/ttyACM* /dev/ttyUSB* /dev/braccio`; are you in `dialout`? |
| Arm does not move, no errors | Is the shield's 5 V supply on? USB alone does not power the servos |
| Arm moves but jerks or resets | Power supply too weak; use 5 V, 4 A or more |
| Nothing for ~8 s after launch | Normal: the UNO restarts and soft-starts the servos |
| Arm keeps its old pose at launch instead of standing up | The board did not restart. `bash scripts/check_uno_serial.sh` tells you; a clone without the reset capacitor cannot be restarted from the PC: press RESET or power-cycle it |
| `Firmware: ERR ...` in the log | Wrong firmware on the board; reflash `braccio_uno_firmware` |

## Arduino UNO Q (alternative)

The older UNO Q set-up is still in the repository: the App Lab apps in
`app_lab/` over the network (`remote.launch.py`), and
`firmware/unoq_braccio_firmware`. See [architecture.md](architecture.md) and
[platform-setup.md](platform-setup.md).
