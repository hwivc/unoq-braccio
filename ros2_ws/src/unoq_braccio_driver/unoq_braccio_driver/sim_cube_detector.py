"""Overhead-camera detector: the authoritative, continuously-running source of
cube and bin locations.

Unlike a one-shot "ask, then wait" detector, this one always runs inference,
on every incoming frame, and only ever trusts a detection that has been seen
consistently for a brief moment (``confirm_window_s``) - a single stray frame
cannot move the arm. It publishes what it currently believes at
``publish_rate_hz``, continuously, whether or not anything asked for it:

    /vision/detect_request            std_msgs/String   optional colour filter
    /vision/cube_target                std_msgs/String   JSON, below
    /vision/overhead/image_detections  sensor_msgs/Image  live boxes for RViz

Result::

    {"cubes": [{"color": "red", "x": 0.20, "y": -0.14, "sector": "pick",
                "confidence": 1.0, "samples": 6}, ...],
     "bins":  {"green": {"x": 0.169, "y": 0.141, "cube_color": "red"}, ...}}

x/y are table coordinates in metres. ``sector`` is ``pick``, a bin name, or
``none``. Bins are found by their own colours (not cube colours).

Finding *where the cubes are* and deciding *what colour each one is* are two
separate steps here on purpose. Where the cubes are comes from a swappable
``CubeBoxDetector`` (default: the Edge Impulse model in the repo root; see
``cube_model_backend.py``) - a single-class "cube" model does not report
colour, so colour is decided afterwards by sampling pixels inside each box.
Swapping in a different model only ever means changing the ``detector_backend``
/ ``model_path`` parameters below; nothing else in this file changes.

``/vision/detect_request`` narrows which colours are *published* - it does not
gate detection any more, since detection never stops. A publish there just
sets which colour(s) count from then on; the rolling history for every colour
keeps building in the background regardless, so switching the filter back
does not need to wait out a fresh window.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from unoq_braccio_driver import braccio_workspace as ws
from unoq_braccio_driver.color_vision import (
    best_color,
    find_blobs,
    image_to_rgb,
    rgb_to_image_msg,
    to_hsv,
)
from unoq_braccio_driver.cube_model_backend import create_cube_detector

# Re-exported for older imports / tests.
pixel_to_table = ws.pixel_to_table

# Fixed draw colours (0-255, RGB order - the published image is rgb8).
_BOX_RGB = {"red": (255, 60, 60), "blue": (70, 120, 255), "yellow": (255, 220, 40)}
_UNKNOWN_RGB = (170, 170, 170)


def cluster_samples(samples, radius):
    """Group (x, y) samples into clusters no wider than ``radius`` from their seed.

    Returns a list of lists of points.
    """
    clusters = []
    for point in samples:
        for cluster in clusters:
            if math.hypot(point[0] - cluster[0][0], point[1] - cluster[0][1]) <= radius:
                cluster.append(point)
                break
        else:
            clusters.append([point])
    return clusters


class SimCubeDetector(Node):
    def __init__(self) -> None:
        super().__init__("sim_cube_detector")
        cam_x, cam_y, cam_z = ws.CAMERA_XYZ
        self.declare_parameter("camera_x", cam_x)
        self.declare_parameter("camera_y", cam_y)
        self.declare_parameter("camera_z", cam_z)
        # How long a cube must be seen before it is trusted, and how often the
        # current belief is published. Shorter = more responsive but flakier;
        # longer = steadier but slower to notice a cube that just arrived.
        self.declare_parameter("confirm_window_s", 0.5)
        self.declare_parameter("min_samples", 3)
        self.declare_parameter("cluster_mm", 12.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        # Cube-finding backend. See cube_model_backend.py: "edge_impulse" (default)
        # or "color_blob". model_path="" searches the repo for a .lite/.tflite file.
        self.declare_parameter("detector_backend", "edge_impulse")
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_conf", 0.3)
        self.declare_parameter("model_iou", 0.45)
        # How much of a box must match a colour range to accept that colour.
        self.declare_parameter("color_min_frac", 0.15)
        self.declare_parameter("publish_annotated", True)

        self.info = None
        self.color_filter = ""
        self.cube_detector = None  # built lazily, once camera_info gives us fx/height
        self.cube_history = {}   # colour -> [(t, x, y), ...], newest last
        self.bin_history = {}    # bin name -> [(t, x, y), ...]

        self.create_subscription(CameraInfo, "/vision/overhead/camera_info", self.on_info, 10)
        self.create_subscription(Image, "/vision/overhead/image_raw", self.on_image, 5)
        self.create_subscription(String, "/vision/detect_request", self.on_request, 10)
        self.publisher = self.create_publisher(String, "/vision/cube_target", 10)
        self.annotated_pub = self.create_publisher(Image, "/vision/overhead/image_detections", 5)

        rate = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self.publish_confirmed)

    def on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def on_request(self, msg: String) -> None:
        self.color_filter = msg.data.strip().lower()
        self.get_logger().info(f"Publishing filter set to '{self.color_filter or 'all'}'")

    def on_image(self, msg: Image) -> None:
        if self.info is None:
            return
        rgb = image_to_rgb(msg)
        if rgb is None:
            self.get_logger().warning(f"Unsupported image encoding {msg.encoding}")
            return
        hsv = to_hsv(rgb)

        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        cam_x = float(self.get_parameter("camera_x").value)
        cam_y = float(self.get_parameter("camera_y").value)
        cam_z = float(self.get_parameter("camera_z").value)

        if self.cube_detector is None:
            self.cube_detector = create_cube_detector(
                backend=str(self.get_parameter("detector_backend").value),
                model_path=str(self.get_parameter("model_path").value),
                conf=float(self.get_parameter("model_conf").value),
                iou=float(self.get_parameter("model_iou").value),
                cube_size_m=ws.CUBE_SIZE,
                camera_fx=fx,
                camera_height_m=cam_z - ws.CUBE_CENTRE_Z,
                logger=self.get_logger(),
            )

        publish_annotated = bool(self.get_parameter("publish_annotated").value)
        annotated = rgb.copy() if publish_annotated else None
        now = time.monotonic()
        min_frac = float(self.get_parameter("color_min_frac").value)

        # The model (or the colour-blob fallback) only says "a cube is here";
        # colour comes from sampling pixels inside the box it returned. Every
        # colour is recorded regardless of the publishing filter, so changing
        # the filter later needs no fresh window to build up history again.
        for x1, y1, x2, y2, score in self.cube_detector.find_cubes(rgb):
            xi1, yi1 = max(0, int(x1)), max(0, int(y1))
            xi2, yi2 = min(rgb.shape[1], int(round(x2))), min(rgb.shape[0], int(round(y2)))
            color, frac = best_color(hsv[yi1:yi2, xi1:xi2], ws.CUBE_HSV)
            confident = color is not None and frac >= min_frac
            if confident:
                u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                x, y = ws.pixel_to_table(
                    u, v, fx, fy, cx, cy, cam_x, cam_y, cam_z - ws.CUBE_CENTRE_Z
                )
                self.cube_history.setdefault(color, []).append((now, x, y))
            if annotated is not None:
                self._draw_box(annotated, (xi1, yi1, xi2, yi2), score, color if confident else None)

        for bin_ in ws.BINS:
            bin_px = bin_.size * fx / (cam_z - bin_.height)
            for u, v, _ in find_blobs(
                hsv, ws.BIN_HSV[bin_.name], 0.5 * bin_px ** 2, 1.6 * bin_px ** 2
            ):
                x, y = ws.pixel_to_table(u, v, fx, fy, cx, cy, cam_x, cam_y, cam_z - bin_.height)
                self.bin_history.setdefault(bin_.name, []).append((now, x, y))

        if annotated is not None:
            self.annotated_pub.publish(rgb_to_image_msg(annotated, msg.header))

    def _draw_box(self, annotated, box, score, color) -> None:
        import cv2

        x1, y1, x2, y2 = box
        rgb_color = _BOX_RGB.get(color, _UNKNOWN_RGB)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), rgb_color, 2)
        label = f"{color} {score:.2f}" if color else f"cube? {score:.2f}"
        cv2.putText(annotated, label, (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, rgb_color, 1, cv2.LINE_AA)

    def publish_confirmed(self) -> None:
        now = time.monotonic()
        window = float(self.get_parameter("confirm_window_s").value)
        min_samples = int(self.get_parameter("min_samples").value)
        radius = float(self.get_parameter("cluster_mm").value) / 1000.0

        cubes = []
        for color, history in self.cube_history.items():
            history[:] = [s for s in history if now - s[0] <= window]  # only a brief moment
            if self.color_filter and color != self.color_filter:
                continue
            points = [(x, y) for _, x, y in history]
            for cluster in cluster_samples(points, radius):
                if len(cluster) < min_samples:
                    continue  # flicker, not a cube
                x = sum(p[0] for p in cluster) / len(cluster)
                y = sum(p[1] for p in cluster) / len(cluster)
                cubes.append({
                    "color": color,
                    "x": x,
                    "y": y,
                    "sector": ws.sector_of(x, y),
                    "confidence": len(cluster) / max(1, len(points)),
                    "samples": len(cluster),
                })

        bins = {}
        for name, history in self.bin_history.items():
            history[:] = [s for s in history if now - s[0] <= window]
            points = [(x, y) for _, x, y in history]
            if not points:
                continue
            biggest = max(cluster_samples(points, radius), key=len)
            if len(biggest) >= min_samples:
                bins[name] = {
                    "x": sum(p[0] for p in biggest) / len(biggest),
                    "y": sum(p[1] for p in biggest) / len(biggest),
                    "cube_color": ws.BIN_BY_NAME[name].cube_color,
                }

        self.publisher.publish(String(data=json.dumps({"cubes": cubes, "bins": bins})))


def main() -> None:
    rclpy.init()
    node = SimCubeDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
