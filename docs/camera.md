# Real Camera Set-up

The overhead camera finds the cubes; the arm needs their position from its
base. A step-by-step terminal tool, `real_setup`, collects everything and
saves it to `~/.ros/braccio_setup.yaml`. There is no file to edit. Simulation
needs none of this.

## 1. Camera

**Phone (Android "IP Webcam"):** phone and ROS machine on the same WiFi (not
a guest network). In the app set *Video resolution* to **640x480**, turn off
power saving, use the normal 1x camera (not ultra-wide), and **Start server**.
Check `http://<phone-ip>:8080/video` opens in a browser on the ROS machine.

**USB webcam:** use its index, `camera:=0` (`ls /dev/video*` lists them).

Mount it:

- **Looking straight down** at the table (phone held level).
- **Top of the picture pointing the way the arm faces** (away from the arm).
- Firm (tripod, clamp, shelf); it must not move after set-up.
- About 50-70 cm up, with the cubes and drop points in view.

## 2. Set-up

Terminal 1 (arm + camera; the first time it starts without cube detection):

```bash
ros2 launch unoq_braccio_bringup real.launch.py serial_port:=/dev/braccio \
  camera:=http://192.168.1.192:8080/video
```

Terminal 2:

```bash
ros2 run unoq_braccio_driver real_setup
```

1. **Camera direction.** Enter, and the arm leans out. Turn it with `+` / `-`
   (1 degree; `]` / `[` for 5) until it points at the spot on the table right
   under the camera (hang a string from the phone to find it). Enter saves.
2. **Measurements.** Type, in mm: the distance from the centre of the arm
   base to that spot, the camera height (lens to table), and the cube size.
3. **Colours.** Put one cube on the table right under the camera. The
   terminal shows the colour it sees, e.g. `seeing red`. `s` saves it; put
   the next cube; `n` when done. The colour comes from the cube's own pixels
   (not the paper, shadows or any box drawn on screen), and a colour too close
   to one already saved is refused. The cube's known size also measures the
   camera's zoom.
4. **Drop points.** For each colour, the arm moves out; move the gripper over
   where those cubes should go (`+` / `-` turn, `w` / `s` further / nearer)
   and press Enter. Drop points must be at least 8 cm apart.
5. **Touch calibration** (makes picks accurate). Put one cube anywhere on the
   table and press Enter: the arm moves over where it thinks the cube is.
   Nudge it (`+` / `-` turn, `w` / `s` further / nearer by 5 mm, `W` / `S`
   by 2 mm for the last bit, or single joints: `j`/`l` base, `i`/`k`
   shoulder, `y`/`h` elbow, `t`/`g` wrist, 1 degree per press) until the closed
   fingertips are centred right over the cube, then Enter. Move the cube and
   repeat: at least 4 points, 6-8 spread over where cubes and drop points will
   be is best; `u` undoes a point, `d` when done. It prints the error per
   point; a point much worse than the others was nudged wrong (`u`, redo).
   Because it records where the arm really had to go, it corrects camera
   tilt, picture rotation, the step 1-2 measurements and the arm's own
   errors together. Without it the camera must look exactly straight down.

Redo a single step later (the others are kept):

```bash
ros2 run unoq_braccio_driver real_setup --ros-args -p steps:=touch   # or camera, colors, drops
```

Restart terminal 1 so the detector loads the set-up, then sort the cubes:

```bash
ros2 launch unoq_braccio_bringup real_pick_place.launch.py
ros2 topic echo /vision/cube_target      # optional: where it sees each cube
```

Cubes can go anywhere the arm reaches that is not a drop point. If picks
miss, redo the touch calibration (`steps:=touch`) with more points, spread
over where the misses happen. Redo it whenever the camera or the arm moves.

## Launch options (`real.launch.py`)

| Option | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Arduino UNO USB port (`/dev/braccio` after `bash scripts/setup_uno_serial.sh`) |
| `camera` | `0` | Overhead camera: USB index, `/dev/videoN`, or stream URL |
| `camera_config` / `workspace_config` | `~/.ros/braccio_setup.yaml` | Written by `real_setup` |
| `gripper_camera` | `false` | `true` to use a camera on the gripper |
| `gripper_camera_source` | `1` | Gripper camera: USB index, `/dev/videoN`, or stream URL |
| `camera_width` | `640` | Resize frames to this width (`0` = as is) |
| `speed` | `60` | Arm speed, degrees per second |
| `detector_backend` | `edge_impulse` | Or `color_blob` |
| `rviz` | `true` | Open RViz |

## Troubleshooting

| Problem | Check |
|---|---|
| `No set-up file at ~/.ros/braccio_setup.yaml yet` | Normal the first time: run `real_setup`, then restart `real.launch.py` |
| `No route to host` | PC cannot reach the phone: `ping <phone-ip>`; check the IP in IP Webcam, same WiFi (not guest) |
| `Cannot open camera` | Open the URL in a browser; IP Webcam server running |
| `waiting for camera images` in `real_setup` | `real.launch.py` is not running, or the camera is not connected |
| `no cube seen` | Cube touching the picture edge, or too pale / dark under this light |
| Picture lags / stream drops | Lower the resolution; turn off the phone's power saving |
