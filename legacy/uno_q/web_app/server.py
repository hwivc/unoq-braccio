import argparse
import json
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib import request
from urllib.parse import quote
from urllib.parse import parse_qs, urlparse


JOINT_NAMES = [
    "base",
    "shoulder",
    "elbow",
    "wrist_vertical",
    "wrist_rotation",
    "gripper",
]

JOINT_LIMITS = {
    "base": (0, 180),
    "shoulder": (15, 165),
    "elbow": (0, 180),
    "wrist_vertical": (0, 180),
    "wrist_rotation": (0, 180),
    "gripper": (10, 110),
}

POSES = {
    "rest": [90, 45, 180, 180, 90, 10],
    "ready": [90, 90, 90, 90, 90, 25],
    "grip_test": [90, 90, 90, 90, 90, 95],
    "grip_full": [90, 90, 90, 90, 90, 110],
    "pickup": [90, 70, 45, 80, 90, 100],
    "drop_left": [45, 85, 80, 95, 90, 10],
    "drop_center": [90, 85, 80, 95, 90, 10],
    "drop_right": [135, 85, 80, 95, 90, 10],
    "wave": [60, 90, 90, 60, 120, 25],
}


class BraccioClient:
    def __init__(self, host: str, port: int, timeout: float = 2.0) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout

    def send(self, line: str) -> str:
        with socket.create_connection((self.host, self.port), timeout=self.timeout) as sock:
            sock.sendall((line.strip() + "\n").encode("ascii"))
            return sock.recv(512).decode("ascii", errors="replace").strip()

    def move(self, values: list[int]) -> str:
        return self.send("M " + " ".join(str(value) for value in values))

    def status(self) -> str:
        return self.send("S")


def clamp_pose(values: list[int]) -> list[int]:
    result = []
    for index, name in enumerate(JOINT_NAMES):
        minimum, maximum = JOINT_LIMITS[name]
        value = values[index] if index < len(values) else POSES["rest"][index]
        result.append(max(minimum, min(maximum, int(value))))
    return result


def parse_status(status: str) -> dict:
    parsed = {"raw": status}
    if not status.startswith("STAT "):
        return parsed
    for part in status.split()[1:]:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        parsed[key] = value
    return parsed


class Handler(BaseHTTPRequestHandler):
    server_version = "UNOQBraccioWeb/0.1"

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.serve_file("index.html", "text/html")
        elif parsed.path == "/app.js":
            self.serve_file("app.js", "application/javascript")
        elif parsed.path == "/styles.css":
            self.serve_file("styles.css", "text/css")
        elif parsed.path == "/api/config":
            self.json_response(
                {
                    "unoq_host": self.server.unoq_host,
                    "control_port": self.server.control_port,
                    "camera_url": self.server.camera_url,
                    "joint_names": JOINT_NAMES,
                    "joint_limits": JOINT_LIMITS,
                    "poses": POSES,
                }
            )
        elif parsed.path == "/api/status":
            self.handle_status()
        elif parsed.path == "/api/cameras":
            self.handle_cameras()
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/move":
            self.handle_move()
        elif parsed.path == "/api/pose":
            self.handle_pose()
        elif parsed.path == "/api/stop":
            self.handle_stop()
        elif parsed.path == "/api/camera":
            self.handle_camera_select()
        else:
            self.send_error(404)

    def read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def client(self) -> BraccioClient:
        return BraccioClient(self.server.unoq_host, self.server.control_port)

    def handle_status(self) -> None:
        try:
            status = self.client().status()
            self.json_response({"ok": True, "status": parse_status(status)})
        except OSError as error:
            self.json_response({"ok": False, "error": str(error)}, status=503)

    def handle_cameras(self) -> None:
        try:
            with request.urlopen(self.server.camera_devices_url, timeout=2) as response:
                body = response.read().decode("utf-8", errors="replace")
            lines = [line.strip() for line in body.splitlines() if line.strip()]
            selected = lines[0] if lines else ""
            devices = []
            for line in lines:
                if line not in devices:
                    devices.append(line)
            self.json_response({"ok": True, "selected": selected, "devices": devices})
        except OSError as error:
            self.json_response({"ok": False, "error": str(error)}, status=503)

    def handle_move(self) -> None:
        payload = self.read_json()
        values = clamp_pose([int(value) for value in payload.get("values", [])])
        try:
            response = self.client().move(values)
            self.json_response({"ok": response == "OK", "response": response, "values": values})
        except OSError as error:
            self.json_response({"ok": False, "error": str(error)}, status=503)

    def handle_pose(self) -> None:
        payload = self.read_json()
        name = str(payload.get("name", ""))
        if name not in POSES:
            self.json_response({"ok": False, "error": f"Unknown pose: {name}"}, status=400)
            return
        values = clamp_pose(POSES[name])
        try:
            response = self.client().move(values)
            self.json_response({"ok": response == "OK", "response": response, "values": values})
        except OSError as error:
            self.json_response({"ok": False, "error": str(error)}, status=503)

    def handle_stop(self) -> None:
        values = clamp_pose(POSES["rest"])
        try:
            response = self.client().move(values)
            self.json_response({"ok": response == "OK", "response": response, "values": values})
        except OSError as error:
            self.json_response({"ok": False, "error": str(error)}, status=503)

    def handle_camera_select(self) -> None:
        payload = self.read_json()
        device = str(payload.get("device", "")).strip()
        if not device:
            self.json_response({"ok": False, "error": "Missing camera device"}, status=400)
            return
        url = f"{self.server.camera_select_url}?device={quote(device)}"
        try:
            req = request.Request(url, method="POST", data=b"")
            with request.urlopen(req, timeout=3) as response:
                body = response.read().decode("utf-8", errors="replace").strip()
            self.json_response({"ok": True, "response": body, "camera_url": self.server.camera_url})
        except OSError as error:
            self.json_response({"ok": False, "error": str(error)}, status=503)

    def serve_file(self, filename: str, content_type: str) -> None:
        path = Path(__file__).with_name("static") / filename
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def json_response(self, payload: dict, status: int = 200) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, format: str, *args) -> None:
        print(f"{self.address_string()} - {format % args}")


class WebServer(ThreadingHTTPServer):
    def __init__(self, server_address, handler, unoq_host, control_port, camera_url, camera_base_url):
        super().__init__(server_address, handler)
        self.unoq_host = unoq_host
        self.control_port = control_port
        self.camera_url = camera_url
        self.camera_devices_url = f"{camera_base_url}/camera-devices"
        self.camera_select_url = f"{camera_base_url}/camera-select"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--unoq-host", default="192.168.1.64")
    parser.add_argument("--control-port", type=int, default=8765)
    parser.add_argument("--camera-url", default="")
    args = parser.parse_args()

    camera_base_url = f"http://{args.unoq_host}:8080"
    camera_url = args.camera_url or f"{camera_base_url}/stream"
    server = WebServer(
        (args.host, args.port),
        Handler,
        unoq_host=args.unoq_host,
        control_port=args.control_port,
        camera_url=camera_url,
        camera_base_url=camera_base_url,
    )
    print(f"Web control: http://{args.host}:{args.port}")
    print(f"UNO Q control: {args.unoq_host}:{args.control_port}")
    print(f"Camera: {camera_url}")
    server.serve_forever()


if __name__ == "__main__":
    main()
