from arduino.app_utils import App, Bridge
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse
import os
import socket
import threading
import time

try:
    import cv2
except Exception as exc:
    cv2 = None
    CV2_IMPORT_ERROR = exc
else:
    CV2_IMPORT_ERROR = None


CONTROL_PORT = 8765
CAMERA_PORT = 8080
DEFAULT_CAMERA_DEVICE = os.environ.get("BRACCIO_CAMERA_DEVICE", "/dev/video4")
START_TIME = time.monotonic()

frame_lock = threading.Lock()
camera_lock = threading.Lock()
latest_jpeg = None
camera_message = "camera starting"
camera_device = DEFAULT_CAMERA_DEVICE
camera_generation = 0
move_count = 0
last_move_ms = 0
last_command_ms = 0
last_target = [90, 45, 180, 180, 90, 10]


def handle_arm_command(command):
    global move_count, last_move_ms, last_command_ms, last_target

    parts = command.strip().split()
    if len(parts) == 1 and parts[0] == "S":
        uptime_ms = int((time.monotonic() - START_TIME) * 1000)
        target = ",".join(str(value) for value in last_target)
        return (
            f"STAT uptime_ms={uptime_ms} move_count={move_count} "
            f"last_move_ms={last_move_ms} last_command_ms={last_command_ms} "
            f"target={target}"
        )

    if len(parts) != 7 or parts[0] != "M":
        return "ERR"

    try:
        values = [int(value) for value in parts[1:]]
    except ValueError:
        return "ERR"

    start = time.monotonic()
    result = Bridge.call("move_braccio", *values)
    last_move_ms = int((time.monotonic() - start) * 1000)
    last_command_ms = int((time.monotonic() - START_TIME) * 1000)
    move_count += 1
    last_target = values
    return "OK" if result is None or result is True else str(result)


def arm_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", CONTROL_PORT))
        server.listen(4)
        print(f"Braccio control listening on {CONTROL_PORT}")
        while True:
            client, address = server.accept()
            with client:
                data = client.recv(128).decode("ascii", errors="replace")
                response = handle_arm_command(data)
                client.sendall((response + "\n").encode("ascii"))
                print(f"{address[0]}: {data.strip()} -> {response}")


def list_camera_devices():
    devices = [str(path) for path in sorted(Path("/dev").glob("video*"))]
    return devices


def open_camera(device_name):
    if device_name.startswith("/dev/"):
        capture = cv2.VideoCapture(device_name)
    else:
        try:
            capture = cv2.VideoCapture(int(device_name))
        except ValueError:
            capture = cv2.VideoCapture(device_name)

    if capture.isOpened():
        ok, _ = capture.read()
        if ok:
            return capture
    capture.release()
    return None


def find_camera(device_name):
    preferred = Path(device_name)
    if preferred.exists():
        capture = open_camera(str(preferred))
        if capture is not None:
            print(f"Using configured camera {preferred}")
            return capture
        print(f"Configured camera {preferred} did not produce frames")

    for device in sorted(Path("/dev").glob("video*")):
        if str(device) == str(preferred):
            continue
        capture = open_camera(str(device))
        if capture is not None:
            print(f"Using camera {device}")
            return capture

    for index in range(4):
        capture = open_camera(str(index))
        if capture is not None:
            print(f"Using camera index {index}")
            return capture

    return None


def camera_loop():
    global latest_jpeg, camera_message
    if cv2 is None:
        camera_message = f"OpenCV unavailable: {CV2_IMPORT_ERROR}"
        print(f"OpenCV unavailable; camera stream disabled: {CV2_IMPORT_ERROR}")
        return

    while True:
        with camera_lock:
            selected = camera_device
            generation = camera_generation
        capture = find_camera(selected)
        if capture is None:
            camera_message = f"No usable camera found for {selected}"
            with frame_lock:
                latest_jpeg = None
            print(camera_message)
            time.sleep(2)
            continue

        camera_message = f"camera online: {selected}"
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        capture.set(cv2.CAP_PROP_FPS, 15)

        while True:
            with camera_lock:
                if generation != camera_generation:
                    break
            ok, frame = capture.read()
            if not ok:
                time.sleep(0.2)
                continue
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                with frame_lock:
                    latest_jpeg = encoded.tobytes()
            time.sleep(0.02)

        capture.release()
        with frame_lock:
            latest_jpeg = None


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path not in ("/", "/stream", "/camera-status", "/camera-devices"):
            self.send_error(404)
            return

        if parsed.path == "/camera-status":
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(camera_message.encode("utf-8"))
            return

        if parsed.path == "/camera-devices":
            devices = list_camera_devices() if cv2 is not None else []
            with camera_lock:
                selected = camera_device
            body = ("\n".join([selected] + devices) + "\n").encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            if cv2 is None:
                self.wfile.write(b"UNO Q Braccio web agent. Control online; camera disabled because OpenCV is unavailable.\n")
            else:
                self.wfile.write(b"UNO Q Braccio web agent. Use /stream\n")
            return

        if cv2 is None:
            self.send_response(503)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"Camera disabled because OpenCV is unavailable.\n")
            return

        with frame_lock:
            first_frame = latest_jpeg
        if first_frame is None:
            self.send_response(503)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(f"Camera unavailable: {camera_message}\n".encode("utf-8"))
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        while True:
            with frame_lock:
                frame = latest_jpeg
            if frame is None:
                break
            try:
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(0.07)
            except (BrokenPipeError, ConnectionResetError):
                break

    def do_POST(self):
        global camera_device, camera_generation, camera_message, latest_jpeg
        parsed = urlparse(self.path)
        if parsed.path != "/camera-select":
            self.send_error(404)
            return

        query = parse_qs(parsed.query)
        device = query.get("device", [""])[0].strip()
        if not device:
            length = int(self.headers.get("Content-Length", "0"))
            device = self.rfile.read(length).decode("utf-8").strip() if length else ""
        if not device:
            self.send_error(400, "missing camera device")
            return

        with camera_lock:
            camera_device = device
            camera_generation += 1
        with frame_lock:
            latest_jpeg = None
        camera_message = f"switching camera to {device}"
        body = f"OK {device}\n".encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        return


def camera_server():
    server = ThreadingHTTPServer(("0.0.0.0", CAMERA_PORT), StreamHandler)
    print(f"USB camera stream listening on {CAMERA_PORT}")
    server.serve_forever()


def loop():
    threading.Thread(target=arm_server).start()
    if cv2 is None:
        print(f"OpenCV unavailable; starting control without camera: {CV2_IMPORT_ERROR}")
    else:
        threading.Thread(target=camera_loop).start()
    threading.Thread(target=camera_server).start()
    while True:
        time.sleep(1)


App.run(user_loop=loop)
