# Braccio Web Agent

Run this single App Lab app on the UNO Q for the web dashboard.

It combines:

- TCP arm control on port `8765`
- USB camera MJPEG stream on port `8080`

Use this app instead of running `braccio_remote_agent` and
`usb_camera_streamer` separately.

Endpoints:

```text
tcp://<UNO_Q_IP_ADDRESS>:8765
http://<UNO_Q_IP_ADDRESS>:8080/stream
http://<UNO_Q_IP_ADDRESS>:8080/camera-devices
http://<UNO_Q_IP_ADDRESS>:8080/camera-status
```

The default camera device is `/dev/video4`, based on testing on this UNO Q. To
override it, set `BRACCIO_CAMERA_DEVICE` before starting the app if App Lab
exposes environment configuration.

You can also switch the camera while the app is running:

```bash
curl http://127.0.0.1:8080/camera-devices
curl -X POST "http://127.0.0.1:8080/camera-select?device=/dev/video0"
```

The web dashboard exposes this as a Camera selector plus a custom device field.

## Start It

`app_lab/braccio_web_agent` is a folder, not a PowerShell command. Do not run
this from PowerShell:

```powershell
app_lab/braccio_web_agent
```

Use Arduino App Lab:

1. Open Arduino App Lab.
2. Connect to the UNO Q.
3. Create or open an app named `braccio_web_agent`.
4. Replace the generated files with this folder's contents.
5. Add the `Servo` dependency if App Lab asks. The `Arduino_RouterBridge`
   library is provided by recent `arduino:zephyr` platforms. This app uses
   direct Servo control for the Braccio shield pins so it does not require the
   desktop `Braccio` library inside App Lab.

Camera streaming uses OpenCV only if it is already available in the UNO Q Python
environment. If OpenCV is missing, arm control still starts and the camera
endpoint reports that the camera is disabled.
6. Run the app.

If App Lab says it cannot import `python\__pycache__`, remove generated cache
folders before importing. From the repo root:

```powershell
Get-ChildItem -Path .\app_lab -Recurse -Directory -Filter __pycache__ | Remove-Item -Recurse -Force
```

Or create a clean import ZIP:

```powershell
.\scripts\package_app_lab.ps1 braccio_web_agent
```

Then import the ZIP from `dist/`. Do not import `app_lab/braccio_web_agent.zip`
if one exists from an old manual zip operation; that archive may contain
`__pycache__` and fail App Lab validation.

Then test from PowerShell:

```powershell
python -c "import socket; s=socket.create_connection(('192.168.1.64',8765),timeout=3); s.sendall(b'S\n'); print(s.recv(512).decode()); s.close()"
Invoke-WebRequest -UseBasicParsing http://192.168.1.64:8080/
```

Expected:

```text
STAT uptime_ms=...
UNO Q Braccio web agent. Use /stream
```

Camera diagnostics over SSH:

```bash
ls -l /dev/video*
curl http://127.0.0.1:8080/camera-status
curl http://127.0.0.1:8080/camera-devices
```

If you have SSH and `arduino-app-cli` enabled on the UNO Q, you can copy the
folder and start it remotely:

```powershell
scp -r .\app_lab\braccio_web_agent arduino@192.168.1.64:~/ArduinoApps/
ssh arduino@192.168.1.64
arduino-app-cli app start ~/ArduinoApps/braccio_web_agent
arduino-app-cli app logs ~/ArduinoApps/braccio_web_agent
```
