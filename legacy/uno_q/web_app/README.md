# Braccio Web Control

Browser dashboard for controlling the UNO Q Braccio over the network.

It talks directly to:

- `app_lab/braccio_remote_agent` on TCP port `8765`
- `app_lab/usb_camera_streamer` on HTTP port `8080`

Recommended: run `app_lab/braccio_web_agent`, which combines both services in
one App Lab app.

That path is an App Lab project folder, not a shell command. Start it from
Arduino App Lab on the UNO Q, or copy it to the board and start it with
`arduino-app-cli` over SSH if your board is configured for that.

ROS 2 is not required for this web UI.

## Start

```bash
cd web_app
python server.py --host 0.0.0.0 --port 5000 --unoq-host 192.168.1.64
```

Open:

```text
http://localhost:5000
```

The dashboard includes a Camera selector. It reads detected UNO Q devices from
`http://<UNO_Q_IP_ADDRESS>:8080/camera-devices` and can switch the stream to a
different device such as `/dev/video0`, `/dev/video1`, or `/dev/video4`.

From another device on the same network:

```text
http://<PC_IP_ADDRESS>:5000
```

## Required UNO Q Apps

Run this App Lab app for arm control:

```text
app_lab/braccio_remote_agent
```

Run this App Lab app for camera view:

```text
app_lab/usb_camera_streamer
```
