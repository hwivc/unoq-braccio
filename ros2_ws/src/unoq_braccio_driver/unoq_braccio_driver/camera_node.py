"""One camera node for a USB webcam or a WiFi / IP camera stream.

    ros2 run unoq_braccio_driver camera_node --ros-args -p camera:=0
    ros2 run unoq_braccio_driver camera_node --ros-args -p camera:=http://192.168.1.192:8080/video

``camera`` is a USB device index ("0", "1", ...), a device path
("/dev/video2"), or a stream URL (http://, https://, rtsp://). For the Android
"IP Webcam" app use ``http://<phone-ip>:8080/video``.

Frames are read in a background thread and only the newest one is kept.
Network streams buffer: reading them slower than they send makes the picture
fall seconds behind, which is fatal when the arm acts on it. Publishing the
newest frame at ``fps`` keeps the delay to one frame.

If the camera cannot be opened, or the stream drops (phone screen off, WiFi
hiccup), the node keeps retrying instead of exiting.
"""

import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def parse_source(camera: str):
    """USB index as int, anything else (device path, URL) unchanged."""
    camera = str(camera).strip()
    return int(camera) if camera.isdigit() else camera


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

        self.source = parse_source(self.get_parameter("camera").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = int(self.get_parameter("width").value)
        topic = str(self.get_parameter("topic").value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, topic, 5)
        self.lock = threading.Lock()
        self.latest = None
        self.latest_id = 0
        self.published_id = 0
        self.running = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

        fps = max(1.0, float(self.get_parameter("fps").value))
        self.create_timer(1.0 / fps, self.publish_latest)
        self.get_logger().info(f"Camera {self.source!r} -> {topic}")

    def open(self):
        capture = cv2.VideoCapture(self.source)
        if isinstance(self.source, str) and "://" in self.source:
            # Ask the backend to keep as few frames queued as it can.
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def read_loop(self) -> None:
        capture = None
        reconnect = float(self.get_parameter("reconnect_period").value)
        while self.running:
            if capture is None or not capture.isOpened():
                capture = self.open()
                if not capture.isOpened():
                    self.get_logger().warning(
                        f"Cannot open camera {self.source!r}; retrying",
                        throttle_duration_sec=10.0,
                    )
                    time.sleep(reconnect)
                    continue
                self.get_logger().info(f"Camera {self.source!r} opened")
            ok, frame = capture.read()
            if not ok or frame is None:
                self.get_logger().warning(
                    f"Camera {self.source!r} stopped sending; reconnecting",
                    throttle_duration_sec=10.0,
                )
                capture.release()
                capture = None
                time.sleep(reconnect)
                continue
            with self.lock:
                self.latest = frame
                self.latest_id += 1
        if capture is not None:
            capture.release()

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
