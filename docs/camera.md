# Real Camera: Set-up and Calibration

The overhead camera finds the cubes, and the arm needs each cube's position
in **metres from the arm base**. In simulation that is exact because the camera
pose is known. On the real table, a 5-minute calibration tells the system how
camera pixels map onto your table.

Simulation needs none of this; `sim.launch.py` works as before.

## Do I need to measure the camera height and offsets?

**No.** You do not measure the camera at all: no height, no distance to the
base, no angle. You measure 4-5 **points on the table** instead, and the
software works out the rest.

Why not just measure the camera? To turn pixels into table positions from
measurements you would need the camera's height, its exact position over the
table, its exact tilt in two directions, and the lens focal length. A phone's
focal length is unknown, and a tilt error of only 2 degrees at 60 cm height
already moves every cube by 2 cm, which is enough to miss a 3 cm cube.

Measuring points on the table avoids all of that. From where 4+ known table
points appear in the image, the software fits a *homography*: a 3x3 matrix
that maps any pixel to its table position, whatever the camera's height,
angle or zoom. Measurement errors average out, so with a ruler and careful
marks you get a few millimetres. The camera does not even have to point
straight down.

You also do **not** need a checkerboard lens calibration for this. A normal
phone camera's lens distortion is small over the area the arm reaches. Use
the phone's normal 1x camera, not the ultra-wide one.

The only rule: **after calibrating, the camera must not move or zoom.** If it
does, calibrate again (it takes a few minutes).

## 1. Camera

### Phone as a WiFi camera (Android "IP Webcam")

1. Install **IP Webcam** on the phone and connect it to the same WiFi as the
   ROS machine.
2. In the app: *Video preferences → Video resolution* → **640x480** (enough
   for detection, and less WiFi lag). Turn off *power saving* / screen-off so
   the stream does not stop.
3. Tap **Start server**. The app shows an address such as
   `http://192.168.1.192:8080`.
4. Check the video in a browser: `http://192.168.1.192:8080/video`.
5. Use that `/video` URL as `camera:=` below.

### USB webcam

Use its index: `camera:=0` (or `1`, `2`, ... or `/dev/video2`).
`ls /dev/video*` lists them.

### Placing the camera

- Mount it **firmly** (tripod, clamp, shelf) roughly above the pick area,
  about 50-70 cm up. It may tilt; it must not wobble.
- The whole pick area, the bins and all calibration marks must be **well
  inside** the picture, not at the edges.
- Even, steady lighting. Avoid strong shadows and direct sun.

## 2. Measure and mark the table

Edit `ros2_ws/src/unoq_braccio_bringup/config/real_workspace.yaml`. Every
position is in metres from the **centre of the arm's base**:

```text
           +x  (forward: where the arm points at base = 90)
            ^
            |
 +y <----[ARM]      (seen from above; +y is the arm's left)
```

1. **`cube_size`**: your cube's edge length (0.03 = 3 cm).
2. **`calibration_points`**: 4-5 points spread over the area where cubes will
   be (not in a line). Mark each on the table with tape or a pen: measure x
   forward from the base centre, then y sideways. The defaults are fine for a
   standard Braccio set-up.
3. **`pick_area`**: the rectangle where cubes to be sorted are placed.
4. **`bins`**: measure the centre of each bin and set which cube colour goes
   there. Bins are not looked for by the camera on the real arm; these
   positions are used directly.
5. **`gripper_closed`**: tune later with `manual_control`
   ([manual_control.md](manual_control.md)).

Rebuild after editing (`colcon build`), or pass the file directly with
`workspace_config:=/path/to/real_workspace.yaml`.

## 3. Calibrate

Terminal 1, start the arm and camera:

```bash
ros2 launch unoq_braccio_bringup real.launch.py camera:=http://192.168.1.192:8080/video
```

Terminal 2, start the calibration window (needs a screen):

```bash
ros2 run unoq_braccio_driver table_calibration
```

(It reads the installed `real_workspace.yaml`; add
`--ros-args -p workspace_config:=/path/to/file.yaml` to use another one.)

For each point the window asks for, put **one** cube centred on that mark:

| Key | Action |
|---|---|
| `SPACE` | Accept the cube the tool found (green cross) |
| left click | Use the clicked pixel instead (click the cube's centre) |
| `u` | Undo the last point |
| `q` / `Esc` | Quit |

After the last point it prints the error for each point and saves
`~/.ros/braccio_table_calibration.yaml`. **RMS under 5 mm is good.** If one
point is much worse than the others, its mark was probably measured wrong:
press `u` and redo it.

The window then switches to **check mode**: move a cube anywhere and its
measured x, y is shown live. Compare with a ruler. Quit with `q`.

The running detector reads the calibration when it starts, so restart
`real.launch.py` after calibrating.

## 4. Check detection

In RViz (opened by `real.launch.py`), the overhead camera panel shows boxes
around detected cubes, and the 3D view shows them on the table.

```bash
ros2 topic echo /vision/cube_target      # positions in metres, and which sector
```

If a cube is not boxed, or gets the wrong colour, adjust its range under
`cube_hsv` in `real_workspace.yaml`, or try the colour fallback
(`detector_backend:=color_blob`).

## 5. Pick and place

```bash
ros2 launch unoq_braccio_bringup real_pick_place.launch.py
```

## Launch options (`real.launch.py`)

| Option | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Arduino UNO USB port (`/dev/braccio` after `scripts/setup_uno_serial.sh`) |
| `speed` | `60` | Arm speed, degrees per second |
| `camera` | `0` | Overhead camera: USB index, `/dev/videoN`, or stream URL |
| `camera_width` | `640` | Resize frames to this width (`0` = as is) |
| `gripper_camera` | `false` | `true` to use a camera on the gripper for pick checks |
| `gripper_camera_source` | `1` | Gripper camera: USB index, `/dev/videoN`, or stream URL |
| `workspace_config` | package `config/real_workspace.yaml` | Your workspace file |
| `calibration_file` | `~/.ros/braccio_table_calibration.yaml` | Written by `table_calibration` |
| `detector_backend` | `edge_impulse` | Or `color_blob` |
| `rviz` | `true` | Open RViz |

Use the same `gripper_camera` value for `real_pick_place.launch.py`.

## Troubleshooting

| Problem | Check |
|---|---|
| `Cannot open camera` | Open the URL in a browser; phone and PC on the same WiFi; IP Webcam server running |
| Picture lags seconds behind | Lower the resolution in IP Webcam; the node already keeps only the newest frame |
| Stream keeps dropping | Disable the phone's power saving / screen-off; keep it charging |
| `No usable camera calibration` | Run `table_calibration`, then restart `real.launch.py` |
| Calibration window: "No cube seen" | Cube cut off at the image edge (move the camera), or colour outside `cube_hsv` |
| Positions are off by a constant amount | Base-centre measurement wrong: re-measure the marks from the exact centre of the base |
| Positions are good in the middle, worse at the edges | Add calibration points nearer the edges (still well inside the picture) |
