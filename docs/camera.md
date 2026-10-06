# Real Camera Set-up

The overhead camera finds the cubes; the arm needs their position in metres
from its base. You give the camera's position with a tape measure in one
small file, `ros2_ws/src/unoq_braccio_bringup/config/real_camera.yaml`.
Simulation needs none of this.

## 1. Camera

**Phone (Android "IP Webcam"):** phone and ROS machine on the same WiFi (not
a guest network). In the app set *Video resolution* to **640x480**, turn off
power saving, use the normal 1x camera (not ultra-wide), and **Start server**.
Check `http://<phone-ip>:8080/video` opens in a browser on the ROS machine.

**USB webcam:** use its index, `camera:=0` (`ls /dev/video*` lists them).

## 2. Mount it

- **Looking straight down** at the table (phone held level).
- **Top of the picture pointing the way the arm faces** (away from the arm).
- Firm (tripod, clamp, shelf); it must not move after you measure.
- About 50-70 cm up, with the cubes and bins in view.

## 3. Measure (millimetres)

Edit `real_camera.yaml`:

```yaml
camera_height_mm: 600   # lens straight down to the table
camera_x_mm: 200        # point under the lens: forward from the base centre
camera_y_mm: 0          #   ...and left (+) / right (-) of it
cube_size_mm: 30        # edge length of your cubes
```

Hang a string or use a set square to find the point on the table right below
the lens, then measure from the **centre of the arm base** to that point:
`x` along the direction the arm faces, `y` sideways.

The camera's zoom is measured automatically: at start-up, put **one cube
near the middle of the picture** until the log says
`Camera scale measured from the cubes: ... px`. Paste that number into the
file as `camera_fx_px: ...` to skip this next time.

Bin positions, gripper and cube colours are in `real_workspace.yaml` (same
folder). Rebuild after editing (`colcon build`).

## 4. Run

```bash
# Terminal 1: arm + camera + detection + RViz (includes hardware.launch.py; don't run both)
ros2 launch unoq_braccio_bringup real.launch.py serial_port:=/dev/braccio \
  camera:=http://192.168.1.192:8080/video

# Check positions against a ruler
ros2 topic echo /vision/cube_target

# Terminal 2: sort the cubes
ros2 launch unoq_braccio_bringup real_pick_place.launch.py
```

If the arm lands consistently off in one direction, re-measure `camera_x_mm`
/ `camera_y_mm`; if errors grow towards the edges of the picture, the camera
is not looking straight down.

## Launch options (`real.launch.py`)

| Option | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Arduino UNO USB port (`/dev/braccio` after `bash scripts/setup_uno_serial.sh`) |
| `camera` | `0` | Overhead camera: USB index, `/dev/videoN`, or stream URL |
| `camera_config` | package `config/real_camera.yaml` | Camera measurements and cube size |
| `workspace_config` | package `config/real_workspace.yaml` | Bins, gripper, cube colours |
| `gripper_camera` | `false` | `true` to use a camera on the gripper |
| `gripper_camera_source` | `1` | Gripper camera: USB index, `/dev/videoN`, or stream URL |
| `camera_width` | `640` | Resize frames to this width (`0` = as is) |
| `speed` | `60` | Arm speed, degrees per second |
| `detector_backend` | `edge_impulse` | Or `color_blob` |
| `rviz` | `true` | Open RViz |

## Troubleshooting

| Problem | Check |
|---|---|
| `No route to host` | PC cannot reach the phone: `ping <phone-ip>`; check the IP in IP Webcam, same WiFi (not guest) |
| `Cannot open camera` | Open the URL in a browser; IP Webcam server running |
| Picture lags | Lower the resolution in IP Webcam |
| Stream drops | Turn off the phone's power saving / screen-off |
| `Measuring the camera scale` keeps repeating | Put one cube near the middle of the picture |
| A cube is not boxed / wrong colour | Adjust its range under `cube_hsv` in `real_workspace.yaml` |
