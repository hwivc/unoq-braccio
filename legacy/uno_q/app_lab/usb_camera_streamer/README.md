# UNO Q USB Camera Streamer

Run this App Lab app on the UNO Q when a USB camera is plugged into the UNO Q
and ROS 2 is running on another computer.

It exposes an MJPEG stream:

```text
http://<UNO_Q_IP_ADDRESS>:8080/stream
```

Open that URL in a browser first. If it works there, use
`vision_remote.launch.py` from ROS 2.

