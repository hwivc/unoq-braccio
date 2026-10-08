# UNOQ Braccio Architecture

This document reflects the current repo implementation.

## Main Path: Arduino UNO over USB Serial

```text
ROS 2 host (Linux PC / Raspberry Pi 5)
  -> hardware.launch.py
  -> unoq_braccio_driver serial_bridge
  -> USB serial, 115200 baud, text protocol (../../docs/hardware.md)
  -> Arduino UNO running firmware/braccio_uno_firmware
  -> Servo pins on Braccio shield
```

The bridge publishes the arm's reported position on `/joint_states` and the
raw status line on `/braccio/firmware_status`. Everything below describes the
alternative Arduino UNO Q set-up.

## Runtime Paths (Arduino UNO Q)

### Web Dashboard Path

```text
Browser
  -> web_app/server.py, stdlib ThreadingHTTPServer, port 5000
  -> UNO Q braccio_web_agent, TCP port 8765
  -> Arduino_RouterBridge
  -> MCU sketch
  -> Servo pins on Braccio shield
```

Camera stream:

```text
USB camera on UNO Q
  -> braccio_web_agent Python OpenCV capture, default /dev/video4
  -> MJPEG HTTP stream, port 8080
  -> web dashboard img element
```

If OpenCV or a usable `/dev/video*` device is unavailable, the web agent keeps
arm control online and reports camera status through:

```text
http://<UNO_Q_IP>:8080/camera-status
```

### ROS 2 Path

```text
ROS 2 host
  -> remote.launch.py
  -> unoq_braccio_driver tcp_bridge
  -> UNO Q TCP port 8765
  -> braccio_web_agent / braccio_remote_agent
  -> Arduino_RouterBridge
  -> MCU sketch
  -> Servo pins
```

Topics:

```text
/braccio/joint_command   sensor_msgs/JointState
/braccio/firmware_status std_msgs/String
/braccio/vision_stats    std_msgs/String
/edge_impulse/label      std_msgs/String
```

### Edge Impulse Pick/Place Path

```text
UNO Q camera stream, port 8080
  -> ROS 2 mjpeg_camera_node
  -> /braccio/camera/image_raw
  -> edge_impulse_vision runner command
  -> /edge_impulse/label
  -> pick_place_executor
  -> /braccio/joint_command
  -> tcp_bridge
  -> UNO Q arm control, port 8765
```

## Real Ports

| Port | Owner | Purpose |
| ---: | --- | --- |
| 5000 | `web_app/server.py` on PC | Browser dashboard |
| 8765 | UNO Q App Lab app | Arm control TCP protocol |
| 8080 | UNO Q App Lab app | MJPEG camera stream |

## Real App Lab App

Publish this as the main App Lab app:

```text
app_lab/braccio_web_agent
```

Diagnostic apps:

```text
app_lab/braccio_smoke_test
app_lab/braccio_remote_agent
app_lab/usb_camera_streamer
```

## Hardware Control

The App Lab web agent uses direct `Servo` control on the Braccio shield pins:

| Joint | Pin | Range |
| --- | ---: | --- |
| base | 11 | 0-180 |
| shoulder | 10 | 15-165 |
| elbow | 9 | 0-180 |
| wrist_vertical | 6 | 0-180 |
| wrist_rotation | 5 | 0-180 |
| gripper | 3 | 10-110 |
| soft start | 12 | digital high |

The standalone `firmware/unoq_braccio_firmware` still uses the official
`Braccio` library for direct Arduino CLI firmware flashing.

## Standalone Server Note

A standalone Python server can expose dashboard-compatible ports `8765` and
`8080`, and it can stream `/dev/video4`. It is not a real motor backend unless
it calls a hardware control API. The production motor path in this repo is the
App Lab `braccio_web_agent`, because it uses `Arduino_RouterBridge` to call the
MCU-side `move_braccio` function.
