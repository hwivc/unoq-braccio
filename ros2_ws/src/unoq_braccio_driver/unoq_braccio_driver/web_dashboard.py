"""Web dashboard: watch and drive the arm from any phone, tablet or computer on the LAN.

    ros2 launch unoq_braccio_bringup real.launch.py ... web:=true     # or:
    ros2 run unoq_braccio_driver web_dashboard                        # next to real / sim launch

Then open http://<this-computer>:8000 on any device on the same network. The
first time, the terminal prints a pairing code: enter it in the browser to
create the owner account. Owners invite others from Settings.

The page shows the arm in 3D, drawn from the same URDF and transforms as
RViz, the overhead camera, and what the arm is doing. Whoever has the
controls can sort cubes (learned_pick_demo on the real arm, pick_place_demo
in simulation), move joints, open / close the gripper and go to poses.
Anyone allowed to control can press Stop at any time.

Everything goes through ROS topics (/braccio/joint_command, /joint_states,
/tf, /task/state ...), so it works the same with the real arm and Gazebo.
Python standard library only: no extra installs.
"""

from __future__ import annotations

import json
import mimetypes
import os
import signal
import ssl
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from unoq_braccio_driver import braccio_workspace as ws
from unoq_braccio_driver.braccio_model import JOINT_LIMITS, JOINT_NAMES, POSES, START_POSE
from unoq_braccio_driver.braccio_protocol import parse_status
from unoq_braccio_driver.web_auth import CAN_CONTROL, ROLES, Accounts
from unoq_braccio_driver.web_scene import urdf_scene

STATIC = Path(__file__).with_name("web")
COOKIE = "braccio_session"
TEACH_ROOT = os.path.expanduser("~/.ros/braccio_teach")


def clamp_pose(values, fallback=START_POSE):
    out = []
    for i, name in enumerate(JOINT_NAMES):
        v = values[i] if i < len(values) and values[i] is not None else fallback[i]
        limit = JOINT_LIMITS[name]
        out.append(max(limit.minimum, min(limit.maximum, int(round(float(v))))))
    return out


def list_sessions(root=TEACH_ROOT):
    """Teaching sessions on disk with example / model / drop counts."""
    from unoq_braccio_driver import pick_learning as pl

    out = []
    if not os.path.isdir(root):
        return out
    for name in sorted(os.listdir(root)):
        if not os.path.isdir(os.path.join(root, name)):
            continue
        session = pl.Session(name, root=root)
        out.append({"name": name, "examples": len(session.examples()),
                    "models": session.models(), "drops": sorted(session.drops())})
    return out


# -- the ROS side -------------------------------------------------------------------

class Dashboard(Node):
    def __init__(self):
        super().__init__("web_dashboard")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8000)
        self.declare_parameter("certfile", "")         # HTTPS when both are set
        self.declare_parameter("keyfile", "")
        self.declare_parameter("users_file", "~/.ros/braccio_web.yaml")
        self.declare_parameter("setup_file", "~/.ros/braccio_setup.yaml")
        self.declare_parameter("mode", "auto")          # auto | real | sim
        self.declare_parameter("session", "")           # default teaching session ("" = newest)
        self.declare_parameter("camera_fps", 10.0)

        setup = os.path.expanduser(str(self.get_parameter("setup_file").value))
        if os.path.exists(setup):
            ws.load_config(setup)
        self.accounts = Accounts(str(self.get_parameter("users_file").value))
        self.lock = threading.Lock()

        self.robot_xml = None
        self.scene, self.meshes = None, []
        self.servo = None            # servo degrees now (firmware, or converted joint_states)
        self.target = None           # last command seen on /braccio/joint_command
        self.firmware_at = 0.0
        self.joints_at = 0.0
        self.task_state, self.task_current = "IDLE", {}
        self.frames = {"camera": None, "detections": None}   # (count, Image msg)
        self.jpeg_cache = {}
        self.task = None             # subprocess.Popen of the running demo
        self.task_info = {}
        self.log = deque(maxlen=300)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.command = self.create_publisher(JointState, "/braccio/joint_command", 10)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_subscription(JointState, "/braccio/joint_command", self.on_command, 10)
        self.create_subscription(String, "/braccio/firmware_status", self.on_status, 10)
        self.create_subscription(String, "/task/state", self.on_task_state, 10)
        self.create_subscription(String, "/task/current", self.on_task_current, 10)
        self.create_subscription(Image, "/vision/overhead/image_raw",
                                 lambda m: self.on_image("camera", m), 2)
        self.create_subscription(Image, "/vision/overhead/image_detections",
                                 lambda m: self.on_image("detections", m), 2)

        import tf2_ros

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(3.0, self.fallback_description)
        self.create_timer(0.5, self.reap_task)

    # -- topics ----------------------------------------------------------------

    def on_description(self, msg):
        if msg.data and msg.data != self.robot_xml:
            try:
                scene, meshes = urdf_scene(msg.data)
            except ET.ParseError as error:
                self.get_logger().warning(f"robot_description is not valid URDF: {error}")
                return
            with self.lock:
                self.robot_xml, self.scene, self.meshes = msg.data, scene, meshes
            self.get_logger().info(f"3D model: {len(scene['links'])} links, {len(meshes)} meshes")

    def fallback_description(self):
        """No robot_state_publisher yet: expand the package's xacro ourselves."""
        if self.robot_xml is not None:
            return
        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("unoq_braccio_sim")
            xml = subprocess.run(
                ["xacro", os.path.join(share, "urdf", "braccio.urdf.xacro"),
                 f"mesh_dir:={os.path.join(share, 'meshes', 'braccio_stedden')}"],
                capture_output=True, text=True, timeout=20, check=True).stdout
        except (LookupError, OSError, subprocess.SubprocessError):
            return
        self.on_description(String(data=xml))

    def on_joint_states(self, msg):
        self.joints_at = time.monotonic()
        if time.monotonic() - self.firmware_at < 3.0:
            return  # the real arm reports servo degrees directly
        from unoq_braccio_driver.gui_to_joint_command import urdf_rad_to_servo

        by_name = dict(zip(msg.name, msg.position))
        if all(n in by_name for n in JOINT_NAMES):
            self.servo = clamp_pose([urdf_rad_to_servo(n, by_name[n]) for n in JOINT_NAMES])

    def on_status(self, msg):
        status = parse_status(msg.data)
        if status is not None:
            self.firmware_at = time.monotonic()
            self.servo = list(status["pos"])

    def on_command(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        self.target = clamp_pose([by_name.get(n) for n in JOINT_NAMES], self.target or START_POSE)

    def on_task_state(self, msg):
        self.task_state = msg.data

    def on_task_current(self, msg):
        try:
            self.task_current = json.loads(msg.data)
        except ValueError:
            pass

    def on_image(self, kind, msg):
        count = (self.frames[kind] or (0, None))[0] + 1
        self.frames[kind] = (count, msg)

    # -- state for the browser ---------------------------------------------------

    def mode(self):
        wanted = str(self.get_parameter("mode").value)
        if wanted in ("real", "sim"):
            return wanted
        if time.monotonic() - self.firmware_at < 5.0:
            return "real"
        return "sim" if time.monotonic() - self.joints_at < 5.0 else "offline"

    def link_poses(self):
        import rclpy.time

        scene = self.scene
        if scene is None:
            return {}
        poses = {}
        for link in scene["links"]:
            try:
                t = self.tf_buffer.lookup_transform(scene["root"], link, rclpy.time.Time())
            except Exception:  # noqa: BLE001  (tf raises several types; skip the link)
                continue
            p, q = t.transform.translation, t.transform.rotation
            poses[link] = [round(v, 5) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)]
        return poses

    def snapshot(self, token):
        holder = self.accounts.holder()
        return {
            "mode": self.mode(),
            "servo": self.servo, "target": self.target,
            "links": self.link_poses(),
            "task": {"state": self.task_state, "current": self.task_current,
                     "running": self.task is not None, **self.task_info},
            "control": None if holder is None else {
                "user": holder["user"], "device": holder["device"],
                "mine": holder["token"] == token},
            "cameras": [k for k, v in self.frames.items() if v is not None],
            "time": time.time(),
        }

    def colors(self):
        return list(ws.CUBE_HSV)

    def jpeg(self, kind, max_width=640):
        """Latest frame of ``kind`` as JPEG bytes (encoded once per frame), or None."""
        import cv2

        from unoq_braccio_driver.color_vision import image_to_rgb

        frame = self.frames.get(kind)
        if frame is None:
            return None, 0
        count, msg = frame
        cached = self.jpeg_cache.get(kind)
        if cached and cached[0] == count:
            return cached[1], count
        rgb = image_to_rgb(msg)
        if rgb is None:
            return None, count
        if rgb.shape[1] > max_width:
            scale = max_width / rgb.shape[1]
            rgb = cv2.resize(rgb, (max_width, int(rgb.shape[0] * scale)))
        ok, data = cv2.imencode(".jpg", rgb[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 72])
        if not ok:
            return None, count
        self.jpeg_cache[kind] = (count, data.tobytes())
        return self.jpeg_cache[kind][1], count

    # -- commands -----------------------------------------------------------------

    def send(self, pose):
        pose = clamp_pose(pose, self.target or self.servo or START_POSE)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in pose]
        self.command.publish(msg)
        self.target = pose
        return pose

    def stop(self, who):
        """Stop any running task and hold the arm where it is."""
        self.stop_task(who, reason="stop")
        if self.servo is not None:
            self.send(self.servo)
        self.note(f"STOP by {who}")

    def note(self, line):
        self.log.append(f"{time.strftime('%H:%M:%S')}  {line}")

    def start_task(self, who, colors, drop="auto", session="", model=""):
        if self.task is not None:
            return False, "A task is already running."
        mode = self.mode()
        known = self.colors()
        colors = [c for c in colors if c in known]
        if mode == "real":
            session = session or str(self.get_parameter("session").value) or self.newest_session()
            if not session:
                return False, "No teaching session yet. Teach the arm first (teach_pick)."
            args = ["ros2", "run", "unoq_braccio_driver", "learned_pick_demo", "--ros-args",
                    "-p", f"session:={session}", "-p", f"drop:={drop}",
                    "-p", f"setup_file:={self.get_parameter('setup_file').value}"]
            if colors:
                args += ["-p", f"colors:={','.join(colors)}"]
            if model:
                args += ["-p", f"model:={model}"]
        elif mode == "sim":
            args = ["ros2", "run", "unoq_braccio_driver", "pick_place_demo"]
            if colors:
                args += ["--ros-args", "-p", f"colors:=[{','.join(colors)}]"]
        else:
            return False, "The arm is not connected."
        self.task = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, start_new_session=True)
        self.task_info = {"by": who, "colors": colors or ["all"], "started": time.time(),
                          "session": session if mode == "real" else "", "drop": drop}
        self.task_state, self.task_current = "STARTING", {}
        threading.Thread(target=self.pump_log, args=(self.task,), daemon=True).start()
        self.note(f"{who} started sorting {', '.join(colors) or 'all colours'}")
        return True, "started"

    def pump_log(self, process):
        for line in process.stdout:
            self.log.append(line.rstrip()[:300])

    def stop_task(self, who, reason="stopped"):
        process = self.task
        if process is None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)  # the demo freezes the arm on Ctrl+C
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        self.task = None
        self.task_state = "STOPPED"
        self.note(f"{who} {reason} the task")

    def reap_task(self):
        if self.task is not None and self.task.poll() is not None:
            code = self.task.returncode
            self.task = None
            self.note(f"task finished (exit {code})")
            if self.task_state not in ("COMPLETE", "STOPPED"):
                self.task_state = "COMPLETE" if code == 0 else "FAILED"

    def newest_session(self):
        sessions = [s for s in list_sessions() if s["models"]]
        if not sessions:
            return ""
        return max(sessions, key=lambda s: os.path.getmtime(os.path.join(TEACH_ROOT, s["name"])))["name"]


# -- the HTTP side ------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    server_version = "BraccioDashboard/1.0"
    protocol_version = "HTTP/1.1"

    @property
    def node(self) -> Dashboard:
        return self.server.node

    # helpers
    def token(self):
        cookie = SimpleCookie(self.headers.get("Cookie", ""))
        return cookie[COOKIE].value if COOKIE in cookie else None

    def who(self):
        return self.node.accounts.session(self.token())

    def send_json(self, payload, status=200, headers=()):
        data = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        for k, v in headers:
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def body(self):
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0 or length > 65536:
            return {}
        try:
            return json.loads(self.rfile.read(length))
        except ValueError:
            return {}

    def security_headers(self):
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy",
                         "default-src 'self'; img-src 'self' blob: data:; style-src 'self'; "
                         "script-src 'self'; connect-src 'self'; frame-ancestors 'none'")

    def end_headers(self):
        self.security_headers()
        super().end_headers()

    def log_message(self, fmt, *args):
        if "/api/events" not in self.path and "/api/camera" not in self.path:
            self.node.get_logger().debug(f"{self.address_string()} {fmt % args}")

    # GET
    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/" or path == "/index.html":
            return self.static("index.html")
        if path.startswith("/static/"):
            return self.static(path[len("/static/"):])
        if path == "/api/me":
            return self.api_me()
        user = self.who()
        if user is None:
            return self.send_json({"error": "Sign in first."}, 401)
        if path == "/api/info":
            return self.api_info(user)
        if path == "/api/scene":
            return self.send_json(self.node.scene or {"root": "world", "links": {}})
        if path.startswith("/api/mesh/"):
            return self.api_mesh(path)
        if path == "/api/events":
            return self.api_events()
        if path.startswith("/api/camera/"):
            return self.api_camera(path.rsplit("/", 1)[-1])
        if path == "/api/log" and user["role"] == "owner":
            return self.send_json({"lines": list(self.node.log)[-150:]})
        if path == "/api/review" and user["role"] == "owner":
            return self.api_review()
        if path == "/api/people" and user["role"] == "owner":
            return self.send_json({"people": self.node.accounts.people()})
        self.send_error(404)

    def static(self, name):
        target = (STATIC / name).resolve()
        if STATIC.resolve() not in target.parents or not target.is_file():
            return self.send_error(404)
        data = target.read_bytes()
        kind = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        if target.suffix == ".js":
            kind = "text/javascript"
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache" if target.suffix in (".html", ".js", ".css")
                         else "max-age=86400")
        self.end_headers()
        self.wfile.write(data)

    def api_me(self):
        user = self.who()
        first = self.node.accounts.first_run_code()
        self.send_json({"user": user, "needs_owner": first is not None})

    def api_info(self, user):
        node = self.node
        self.send_json({
            "user": user, "mode": node.mode(), "colors": node.colors(),
            "joints": JOINT_NAMES,
            "limits": {n: [JOINT_LIMITS[n].minimum, JOINT_LIMITS[n].maximum] for n in JOINT_NAMES},
            "poses": {k: v for k, v in POSES.items() if k in ("ready", "rest", "pickup", "drop", "wave")},
            "gripper": {"open": ws.GRIPPER_OPEN, "closed": ws.GRIPPER_CLOSED},
            "sessions": list_sessions(),
            "default_session": str(node.get_parameter("session").value) or node.newest_session(),
        })

    def api_mesh(self, path):
        try:
            index = int(path.rsplit("/", 1)[-1].split(".")[0])
            filename = self.node.meshes[index]
        except (ValueError, IndexError):
            return self.send_error(404)
        data = Path(filename).read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "model/stl")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "max-age=86400")
        self.end_headers()
        self.wfile.write(data)

    def api_events(self):
        """Server-sent events: the arm's state ~15 times a second."""
        token = self.token()
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        try:
            while self.node.accounts.session(token) is not None and not self.server.stopping:
                data = json.dumps(self.node.snapshot(token), separators=(",", ":"))
                self.wfile.write(f"data: {data}\n\n".encode())
                self.wfile.flush()
                time.sleep(1 / 15)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def api_camera(self, kind):
        """MJPEG stream of the overhead camera (``camera``) or the detector's view."""
        kind = kind.split(".")[0]
        if kind not in self.node.frames:
            return self.send_error(404)
        token = self.token()
        fps = max(1.0, float(self.node.get_parameter("camera_fps").value))
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        last = -1
        try:
            while self.node.accounts.session(token) is not None and not self.server.stopping:
                data, count = self.node.jpeg(kind)
                if data is not None and count != last:
                    last = count
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                    self.wfile.flush()
                time.sleep(1.0 / fps)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def api_review(self):
        from unoq_braccio_driver import pick_learning as pl

        name = parse_qs(urlparse(self.path).query).get("session", [""])[0]
        if name not in [s["name"] for s in list_sessions()]:
            return self.send_json({"error": "Unknown session."}, 404)
        session = pl.Session(name, root=TEACH_ROOT)
        examples, tests = session.examples(), session.tests()
        self.send_json({"lines": pl.review_text(examples, tests),
                        "suspects": pl.suspect_examples(examples),
                        "misses": pl.misses(tests, examples)})

    # POST
    def do_POST(self):
        path = urlparse(self.path).path
        # Only our own page can call the API: browsers will not add this header
        # cross-site without a CORS preflight, which this server never allows.
        if self.headers.get("X-Braccio") != "1":
            return self.send_json({"error": "Missing header."}, 403)
        payload = self.body()
        accounts = self.node.accounts
        if path == "/api/login":
            return self.api_login(payload)
        if path == "/api/join":
            ok, message = accounts.join(payload.get("code"), payload.get("name"), payload.get("password"))
            if not ok:
                return self.send_json({"error": message}, 400)
            return self.api_login(payload)
        token = self.token()
        user = accounts.session(token)
        if user is None:
            return self.send_json({"error": "Sign in first."}, 401)
        name = user["user"]
        if path == "/api/logout":
            accounts.logout(token)
            return self.send_json({"ok": True}, headers=[("Set-Cookie", self.cookie("", 0))])
        if user["role"] not in CAN_CONTROL:
            return self.send_json({"error": "You can watch, but not control."}, 403)
        if path == "/api/stop":                       # always allowed
            self.node.stop(name)
            return self.send_json({"ok": True})
        if path == "/api/control/take":
            ok, holder = accounts.take(token, force=bool(payload.get("force")))
            if not ok and holder:
                return self.send_json({"error": f"{holder['user']} is in control.",
                                       "holder": holder["user"]}, 409)
            self.node.note(f"{name} took the controls")
            return self.send_json({"ok": True})
        if path == "/api/control/release":
            accounts.release(token)
            return self.send_json({"ok": True})
        if path.startswith("/api/people") and user["role"] != "owner":
            return self.send_json({"error": "Only an owner can do that."}, 403)
        if path == "/api/people/code":
            role = payload.get("role", "operator")
            if role not in ROLES:
                return self.send_json({"error": "Unknown role."}, 400)
            return self.send_json({"code": accounts.new_code(role), "role": role, "minutes": 10})
        if path == "/api/people/remove":
            if payload.get("name") == name:
                return self.send_json({"error": "You cannot remove yourself."}, 400)
            return self.send_json({"ok": accounts.remove(payload.get("name", ""))})
        if not accounts.driving(token):
            return self.send_json({"error": "Take the controls first."}, 409)
        return self.api_drive(path, payload, name)

    def api_drive(self, path, payload, name):
        node = self.node
        if path == "/api/move":
            values = payload.get("values") or []
            if len(values) != len(JOINT_NAMES):
                return self.send_json({"error": "Need 6 servo values."}, 400)
            return self.send_json({"pose": node.send(values)})
        if path == "/api/pose":
            pose = POSES.get(payload.get("name"))
            if pose is None:
                return self.send_json({"error": "Unknown pose."}, 400)
            return self.send_json({"pose": node.send(pose)})
        if path == "/api/gripper":
            current = list(node.target or node.servo or START_POSE)
            current[5] = ws.GRIPPER_CLOSED if payload.get("close") else ws.GRIPPER_OPEN
            return self.send_json({"pose": node.send(current)})
        if path == "/api/task/start":
            colors = [str(c) for c in payload.get("colors") or []]
            drop = payload.get("drop", "auto")
            if drop not in ("auto", "taught", "side"):
                return self.send_json({"error": "Unknown drop mode."}, 400)
            session = str(payload.get("session", ""))
            if session and session not in [s["name"] for s in list_sessions()]:
                return self.send_json({"error": "Unknown session."}, 400)
            model = str(payload.get("model", ""))
            if model and not model.replace("_", "").isalnum():
                return self.send_json({"error": "Unknown model."}, 400)
            ok, message = node.start_task(name, colors, drop, session, model)
            return self.send_json({"ok": ok, "message": message}, 200 if ok else 409)
        if path == "/api/task/stop":
            node.stop_task(name)
            return self.send_json({"ok": True})
        self.send_error(404)

    def api_login(self, payload):
        device = self.headers.get("User-Agent", "")
        token = self.node.accounts.login(payload.get("name"), payload.get("password"),
                                         self.client_address[0], device_name(device))
        if token is None:
            return self.send_json({"error": "Wrong name or password (or too many tries: wait a minute)."}, 401)
        user = self.node.accounts.session(token)
        self.node.note(f"{user['user']} signed in from {user['device']}")
        return self.send_json({"user": user},
                              headers=[("Set-Cookie", self.cookie(token, int(self.node.accounts.session_s)))])

    def cookie(self, value, max_age):
        secure = "; Secure" if self.server.tls else ""
        return f"{COOKIE}={value}; Max-Age={max_age}; Path=/; HttpOnly; SameSite=Strict{secure}"


def device_name(agent):
    """A short, human device name from a User-Agent ("iPhone", "Mac", ...)."""
    for key, name in (("iPhone", "iPhone"), ("iPad", "iPad"), ("Android", "Android"),
                      ("Macintosh", "Mac"), ("Windows", "Windows PC"), ("CrOS", "Chromebook"),
                      ("Linux", "Linux")):
        if key in agent:
            return name
    return "browser"


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, node, tls):
        super().__init__(address, Handler)
        self.node, self.tls, self.stopping = node, tls, False


def lan_addresses():
    import socket

    addresses = set()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            addresses.add(s.getsockname()[0])
    except OSError:
        pass
    return sorted(addresses) or ["<this-computer>"]


def main():
    rclpy.init()
    node = Dashboard()
    host, port = str(node.get_parameter("host").value), int(node.get_parameter("port").value)
    cert, key = str(node.get_parameter("certfile").value), str(node.get_parameter("keyfile").value)
    server = Server((host, port), node, tls=bool(cert and key))
    if server.tls:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(os.path.expanduser(cert), os.path.expanduser(key))
        server.socket = context.wrap_socket(server.socket, server_side=True)
    scheme = "https" if server.tls else "http"
    threading.Thread(target=server.serve_forever, daemon=True).start()
    urls = ", ".join(f"{scheme}://{a}:{port}" for a in lan_addresses())
    node.get_logger().info(f"Dashboard: open {urls} on any device on this network")
    code = node.accounts.first_run_code()
    if code:
        node.get_logger().warning(f"First time: create the owner account in the browser with pairing code {code}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.stopping = True
        node.stop_task("shutdown", reason="ended")
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
