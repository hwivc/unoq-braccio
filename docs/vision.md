# Vision Options

The easiest way to add sight to the Braccio is a USB camera connected to the
ROS 2 host. It gives normal camera frames, works with OpenCV, and can feed both
Edge Impulse data capture and simple object tracking.

Your current camera placement is close to an eye-in-hand setup: the camera is
mounted above and between the gripper fingers, looking at the object the arm is
about to grab. That is useful for final alignment because the center of the
image can be treated as the gripper's intended pickup point.

## Recommended Path: USB Camera

Install dependencies in your ROS 2 Ubuntu environment:

```bash
sudo apt update
sudo apt install -y ros-jazzy-cv-bridge python3-opencv v4l-utils
```

Check that the camera is visible:

```bash
v4l2-ctl --list-devices
```

Launch the camera and simple color tracker:

```bash
source ros2_ws/install/setup.bash
ros2 launch unoq_braccio_bringup vision_usb.launch.py camera_index:=0 label:=object
```

Topics:

```text
/braccio/camera/image_raw
/braccio/camera/debug
/braccio/vision_stats
/edge_impulse/label
```

The color tracker publishes `/edge_impulse/label` when a target color is
visible. By default it tracks a green object. Tune HSV values with ROS
parameters:

```bash
ros2 run unoq_braccio_driver color_tracker --ros-args \
  -p h_min:=35 -p h_max:=85 \
  -p s_min:=80 -p s_max:=255 \
  -p v_min:=60 -p v_max:=255 \
  -p label:=pickup
```

Run this alongside the Edge Impulse mapper:

```bash
ros2 run unoq_braccio_driver edge_impulse_mapper \
  --ros-args -p mapping_file:=edge_impulse/label_to_pose.yaml
```

## Data Capture with Vision

Run the camera, hardware bridge, and CSV logger together:

```bash
source ros2_ws/install/setup.bash
ros2 launch unoq_braccio_bringup remote.launch.py host:=<UNO_Q_IP_ADDRESS> port:=8765
ros2 launch unoq_braccio_bringup vision_usb.launch.py camera_index:=0 label:=pickup
ros2 launch unoq_braccio_bringup data_capture.launch.py \
  output_file:=edge_impulse/captures/vision_braccio_capture.csv \
  label:=vision
```

The CSV logger records robot command/status data. The color tracker separately
publishes labels and `/braccio/vision_stats` for object visibility and centroid.

## Pixy Camera

Pixy is useful when you want the camera to do object detection onboard and only
send compact object blocks to ROS 2. Use it when you want reliable colored
object tracking without running OpenCV on the host.

Recommended integration:

1. Train signatures in PixyMon.
2. Connect Pixy to the ROS 2 host over USB.
3. Add a Pixy node that publishes detected signatures to `/edge_impulse/label`
   or `/braccio/vision_stats`.

This repo does not include a Pixy driver yet because the USB camera path is
simpler and more general for Edge Impulse image collection.

## ESP32-CAM

ESP32-CAM is useful when you want a cheap wireless camera. It is less ideal for
low-latency arm control because MJPEG streams can lag and Wi-Fi quality varies.

Recommended integration:

1. Flash ESP32-CAM with an MJPEG stream sketch.
2. Confirm the stream opens in a browser.
3. Point an OpenCV capture node at the stream URL.

Example stream URL:

```text
http://<ESP32_CAM_IP>:81/stream
```

Add this after USB camera control is working.

## Remote USB Camera on the UNO Q

Use this when the USB camera is plugged into the UNO Q, but ROS 2 is running on
your PC.

1. Plug the USB camera into the UNO Q.
2. In Arduino App Lab, run `legacy/uno_q/app_lab/usb_camera_streamer` on the UNO Q.
3. Open this URL in a browser:

```text
http://<UNO_Q_IP_ADDRESS>:8080/stream
```

4. In ROS 2, start the remote vision pipeline:

```bash
source ros2_ws/install/setup.bash
ros2 launch unoq_braccio_bringup vision_remote.launch.py \
  stream_url:=http://<UNO_Q_IP_ADDRESS>:8080/stream \
  label:=object
```

This publishes the same topics as the local USB camera path:

```text
/braccio/camera/image_raw
/braccio/camera/debug
/braccio/vision_stats
/edge_impulse/label
```

On the current UNO Q, manual testing found the camera at:

```text
/dev/video4
```

The App Lab `braccio_web_agent` defaults to `/dev/video4` and then scans other
`/dev/video*` devices if that device does not produce frames.

## USB Passthrough to WSL

If the camera is physically attached to Windows but you want ROS 2 in WSL to
use it directly, attach the USB camera with `usbipd-win`.

From elevated PowerShell:

```powershell
usbipd list
usbipd bind --busid <CAMERA_BUSID>
usbipd attach --wsl --busid <CAMERA_BUSID>
```

In Ubuntu WSL:

```bash
ls /dev/video*
v4l2-ctl --list-devices
source ros2_ws/install/setup.bash
ros2 launch unoq_braccio_bringup vision_usb.launch.py camera_index:=0 label:=object
```

Use this route when you want the ROS 2 host to own the camera. Use
`legacy/uno_q/app_lab/usb_camera_streamer` when the UNO Q owns the camera.

## Gripper-Mounted Camera Alignment

The color tracker reports the target centroid as normalized image coordinates:

```text
x_norm=-1.0 left, 0.0 centered, 1.0 right
y_norm=-1.0 top, 0.0 centered, 1.0 bottom
```

Use `/braccio/vision_stats` to manually calibrate first:

```bash
ros2 topic echo /braccio/vision_stats
```

Move an object under the gripper and check whether the target center approaches
`x_norm=0` and `y_norm=0`.

A cautious alignment helper is included, but it is disabled by default:

```bash
source ros2_ws/install/setup.bash
ros2 launch unoq_braccio_bringup vision_assist.launch.py enabled:=false
```

When the color tracker and hardware bridge are both working, enable it:

```bash
ros2 launch unoq_braccio_bringup vision_assist.launch.py enabled:=true
```

The helper makes small base and shoulder nudges to center the detected object.
Keep one hand near servo power while tuning. If the arm moves opposite to what
you expect, reverse the relevant gain:

```bash
ros2 run unoq_braccio_driver visual_servo_assist --ros-args \
  -p enabled:=true \
  -p base_gain_deg:=4.0 \
  -p shoulder_gain_deg:=3.0
```

## Edge Impulse Object Detection

Once the UNO Q camera stream works, use the Edge Impulse pick/place launch file
to run object detection and execute item-specific workflows:

```bash
source ros2_ws/install/setup.bash
ros2 launch unoq_braccio_bringup remote.launch.py host:=192.168.1.64 port:=8765
ros2 launch unoq_braccio_bringup edge_impulse_pick_place.launch.py \
  stream_url:=http://192.168.1.64:8080/stream \
  runner_command:="python3 edge_impulse/runner_template.py --image {image}" \
  workflow_file:=edge_impulse/pick_place_workflows.yaml
```

The runner must print JSON containing `label` and `confidence`. The label is
published to `/edge_impulse/label`; `pick_place_executor` then finds the
matching item in `edge_impulse/pick_place_workflows.yaml` and sends the pose
sequence to `/braccio/joint_command`.
