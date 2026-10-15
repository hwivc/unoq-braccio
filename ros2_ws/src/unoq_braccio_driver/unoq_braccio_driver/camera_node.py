"""One camera node for a USB webcam, a WiFi / IP camera stream or a Grove Vision AI V2.

    ros2 run unoq_braccio_driver camera_node --ros-args -p camera:=0
    ros2 run unoq_braccio_driver camera_node --ros-args -p camera:=http://192.168.1.192:8080/video
    ros2 run unoq_braccio_driver camera_node --ros-args -p camera:=grove:/dev/grove_vision

``camera`` is a USB device index ("0", "1", ...), a device path
("/dev/video2"), or a stream URL (http://, https://, rtsp://). For the Android
"IP Webcam" app use ``http://<phone-ip>:8080/video``.

``grove`` or ``grove:<serial port>`` is a Seeed Grove Vision AI V2 (with its
Raspberry Pi camera) running SSCMA firmware, used as a plain camera: frames
come over USB serial with AT+SAMPLE, no model involved. Bare ``grove`` finds
the board by USB id when exactly one can be it (never the arm's UNO); ports
are never probed, because opening the UNO's port restarts the arm (see
usb_devices). With ``use_device_choices`` (the overhead camera in
real.launch.py) the web page's Settings > Devices can switch to another
camera, or reconnect, while this runs. ``grove_resolution``
picks 0 = 240x240, 1 = 480x480, 2 = 640x480 (default; slower). Needs the
``sscma`` package: pip install -e <ROS--2-Ten-Months-Challenge>/test_camera.

Frames are read in a background thread and only the newest one is kept.
Network streams buffer: reading them slower than they send makes the picture
fall seconds behind, which is fatal when the arm acts on it. Publishing the
newest frame at ``fps`` keeps the delay to one frame.

If the camera cannot be opened, or the stream drops (phone screen off, WiFi
hiccup), the node keeps retrying instead of exiting.
"""

import json
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from unoq_braccio_driver import usb_devices


def parse_source(camera: str):
    """USB index as int, anything else (device path, URL) unchanged."""
    camera = str(camera).strip()
    return int(camera) if camera.isdigit() else camera


def grove_port(source):
    """The serial port of a ``grove`` / ``grove:<port>`` source ("" = find it),
    or None when the source is not a Grove Vision AI V2."""
    if not isinstance(source, str) or not (source == "grove" or source.startswith("grove:")):
        return None
    return source[len("grove:"):].strip()


class SourceChanged(Exception):
    """The web page picked another camera (or asked to reconnect)."""


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_node")
        self.declare_parameter("camera", "0")
        self.declare_parameter("topic", "/vision/overhead/image_raw")
        self.declare_parameter("frame_id", "overhead_camera")
        self.declare_parameter("fps", 15.0)
        # Resize frames to this width (keeping the aspect ratio); 0 = as is.
        # Phone streams are often 1280 or 1920 wide, more than detection needs.
        self.declare_parameter("width", 640)
        self.declare_parameter("reconnect_period", 2.0)
        self.declare_parameter("grove_resolution", 2)   # 0 240x240, 1 480x480, 2 640x480
        # The overhead camera follows the web page's Settings > Devices choice
        # (usb_devices.DEVICES_FILE) and reports how it is doing on /braccio/devices.
        self.declare_parameter("use_device_choices", False)
        self.declare_parameter("devices_file", usb_devices.DEVICES_FILE)

        self.default_source = parse_source(self.get_parameter("camera").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = int(self.get_parameter("width").value)
        topic = str(self.get_parameter("topic").value)
        self.choices = (usb_devices.Choices(str(self.get_parameter("devices_file").value))
                        if bool(self.get_parameter("use_device_choices").value) else None)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, topic, 5)
        self.lock = threading.Lock()
        self.latest = None
        self.latest_id = 0
        self.published_id = 0
        self.running = True
        self.active = None              # (source, reconnect count) being read now
        self.last_check = 0.0
        self.status = {"state": "starting", "detail": "", "port": ""}
        self.frame_times = []
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

        fps = max(1.0, float(self.get_parameter("fps").value))
        self.create_timer(1.0 / fps, self.publish_latest)
        if self.choices is not None:
            self.status_pub = self.create_publisher(String, usb_devices.STATUS_TOPIC, 10)
            self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f"Camera {self.default_source!r} -> {topic}")

    # -- which camera ---------------------------------------------------------------

    def wanted(self):
        """(source, reconnect count): the web page's choice, else the ``camera`` parameter."""
        if self.choices is None:
            return self.default_source, 0
        choices = self.choices.get()
        source = choices["camera"]
        return (self.default_source if source == "auto" else parse_source(source),
                choices["reconnect"]["camera"])

    def check_source(self):
        """Raise SourceChanged when the choice moved on (looked at about once a second)."""
        now = time.monotonic()
        if now - self.last_check < 1.0:
            return
        self.last_check = now
        if self.wanted() != self.active:
            raise SourceChanged()

    def set_status(self, state, detail="", port=""):
        self.status = {"state": state, "detail": detail, "port": port}

    def got_frame(self, frame):
        with self.lock:
            self.latest = frame
            self.latest_id += 1
        now = time.monotonic()
        self.frame_times = [t for t in self.frame_times if now - t < 3.0] + [now]

    def publish_status(self):
        now = time.monotonic()
        recent = [t for t in self.frame_times if now - t < 3.0]
        fps = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 2 else 0.0
        state = self.status["state"]
        if state == "streaming" and (not recent or now - recent[-1] > 3.0):
            state = "stalled"
        self.status_pub.publish(String(data=json.dumps({
            "device": "camera", "source": str(self.active[0] if self.active else self.default_source),
            "port": self.status["port"], "state": state, "detail": self.status["detail"],
            "fps": round(fps, 1), "manual": bool(self.choices and self.choices.get()["camera"] != "auto"),
        })))

    # -- reading ----------------------------------------------------------------------

    def read_loop(self) -> None:
        while self.running:
            previous, self.active = self.active, self.wanted()
            source = self.active[0]
            if previous is None or previous[0] != source:
                self.get_logger().info(f"Camera source: {source!r}")
            try:
                if grove_port(source) is not None:
                    self.read_grove(grove_port(source))
                else:
                    self.read_capture(source)
            except SourceChanged:
                self.get_logger().info("Camera choice changed; switching")
                self.set_status("switching")

    def open(self, source):
        capture = cv2.VideoCapture(source)
        if isinstance(source, str) and "://" in source:
            # Ask the backend to keep as few frames queued as it can.
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while self.running and time.monotonic() < end:
            self.check_source()
            time.sleep(0.1)

    def read_capture(self, source) -> None:
        capture = None
        reconnect = float(self.get_parameter("reconnect_period").value)
        try:
            while self.running:
                self.check_source()
                if capture is None or not capture.isOpened():
                    capture = self.open(source)
                    if not capture.isOpened():
                        self.set_status("searching", f"Cannot open camera {source}", str(source))
                        self.get_logger().warning(
                            f"Cannot open camera {source!r}; retrying",
                            throttle_duration_sec=10.0,
                        )
                        self.wait(reconnect)
                        continue
                    self.get_logger().info(f"Camera {source!r} opened")
                    self.set_status("streaming", "", str(source))
                ok, frame = capture.read()
                if not ok or frame is None:
                    self.set_status("searching", f"Camera {source} stopped sending", str(source))
                    self.get_logger().warning(
                        f"Camera {source!r} stopped sending; reconnecting",
                        throttle_duration_sec=10.0,
                    )
                    capture.release()
                    capture = None
                    self.wait(reconnect)
                    continue
                self.got_frame(frame)
        finally:
            if capture is not None:
                capture.release()

    def read_grove(self, port) -> None:
        """Frames from a Grove Vision AI V2 over USB serial, reconnecting on errors."""
        try:
            from sscma import SSCMAClient, SSCMAError
        except ImportError:
            self.set_status("error", "The sscma package is not installed")
            self.get_logger().error(
                "camera:=grove needs the sscma package: "
                "pip install --break-system-packages -e <ROS--2-Ten-Months-Challenge>/test_camera",
                throttle_duration_sec=60.0)
            self.wait(10.0)
            return
        resolution = int(self.get_parameter("grove_resolution").value)
        reconnect = float(self.get_parameter("reconnect_period").value)
        while self.running:
            self.check_source()
            try:
                device, why = (port, None) if port else usb_devices.find_camera()
                if device is None:
                    raise SSCMAError(why)
                problem = usb_devices.camera_port_problem(device)
                if problem:
                    raise SSCMAError(problem)
                self.set_status("connecting", "", device)
                with SSCMAClient(port=device) as client:
                    client.set_resolution(resolution)
                    self.get_logger().info(f"Grove Vision AI V2 on {device} opened")
                    self.set_status("streaming", "", device)
                    for frame in client.stream(detect=False):
                        self.got_frame(frame.image)
                        if not self.running:
                            break
                        self.check_source()
            except SSCMAError as error:
                self.set_status("searching", str(error), port or "")
                self.get_logger().warning(f"Grove Vision AI V2: {error}; retrying",
                                          throttle_duration_sec=10.0)
                self.wait(reconnect)

    def publish_latest(self) -> None:
        with self.lock:
            frame, frame_id = self.latest, self.latest_id
        if frame is None or frame_id == self.published_id:
            return
        self.published_id = frame_id
        if self.width > 0 and frame.shape[1] != self.width:
            height = int(round(frame.shape[0] * self.width / frame.shape[1]))
            frame = cv2.resize(frame, (self.width, height), interpolation=cv2.INTER_AREA)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def destroy_node(self) -> bool:
        self.running = False
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
