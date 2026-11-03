# 6. See the table

## Objective

A node that reads the overhead camera (a USB webcam, a phone stream or a Grove
Vision AI V2) and publishes the **newest** frame on `/vision/overhead/image_raw`.

## Why we need this now

Everything that follows (finding cubes, teaching, sorting) starts from a picture
of the table. One camera node means every consumer gets the same frames, and only
one program opens the camera.

The docstring explains the main design decision:

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/camera_node.py L1-L29

!!! info "Why only the newest frame?"
    Network streams buffer. If you read slower than they send, the picture falls
    seconds behind, and the arm acts on where a cube *was*. A background thread
    keeps reading and overwriting `latest`; a timer publishes whatever is newest.

## Create

`camera_node.py` in the driver package. Add
`"camera_node = unoq_braccio_driver.camera_node:main",` to the entry points.

## The code

### Which source?

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/camera_node.py parse_source

`"0"` becomes the int `0` (a USB index); anything else stays a string (a path or a URL).

### Set-up

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/camera_node.py CameraNode.__init__

### The reader thread

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/camera_node.py CameraNode.read_loop

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/camera_node.py CameraNode.read_capture

`cv2.VideoCapture` opens a USB index or a URL alike. Any failure leads to a wait and
a retry, so a phone whose screen turns off doesn't kill the node.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/camera_node.py CameraNode.got_frame

### Publishing the newest frame

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/camera_node.py CameraNode.publish_latest

Frames are resized to `width` (default 640), so every later step works in the same pixel scale.

Type the remaining methods (`wanted`, `check_source`, `set_status`,
`publish_status`, `open`, `wait`, `destroy_node`, `main`) from the original. If
you have no Grove board, copy `read_grove` and `grove_port` as-is.

## Run it

```bash
ros2 run unoq_braccio_driver camera_node --ros-args -p camera:=0 \
  -p topic:=/vision/overhead/image_raw
ros2 topic hz /vision/overhead/image_raw          # second terminal
```

Passes when frames arrive at a steady rate. See the picture with `rqt_image_view`, or in RViz.

## What's next

We have pictures but no cubes, just pixels. Next we turn pixels into "a red cube at
(u, v), turned 20°": chapter 7.
