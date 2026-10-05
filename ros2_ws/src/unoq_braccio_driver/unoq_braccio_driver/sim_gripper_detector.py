"""Gripper-camera detector: detection only, never positions.

The gripper camera moves with the arm, so it cannot report where a cube is on
the table (the overhead camera does that). It answers a different question:
"is a cube of this colour in front of the gripper, and how much of the view
does it fill?" The task manager uses that to confirm the right cube before
descending and to confirm a cube is held after grasping.

    /vision/gripper/detect_request   std_msgs/String   cube colour or ""
    /vision/gripper/detection        std_msgs/String   JSON

Result::

    {"request": "red", "colors": {"red": {"area_frac": 0.21, "cx": 0.05, "cy": -0.10}}}

``cx``/``cy`` are the blob centre in [-1, 1] image coordinates. One result is
published per request, from the next frame received.
"""

import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from unoq_braccio_driver import braccio_workspace as ws
from unoq_braccio_driver.color_vision import find_blobs, image_to_rgb, to_hsv


class SimGripperDetector(Node):
    def __init__(self) -> None:
        super().__init__("sim_gripper_detector")
        self.declare_parameter("min_area_frac", 0.005)
        self.declare_parameter("workspace_config", "")  # "" = simulated cube colours
        config = str(self.get_parameter("workspace_config").value)
        if config:
            ws.load_config(os.path.expanduser(config))
        self.pending = None
        self.create_subscription(Image, "/vision/gripper/image_raw", self.on_image, 5)
        self.create_subscription(String, "/vision/gripper/detect_request", self.on_request, 10)
        self.publisher = self.create_publisher(String, "/vision/gripper/detection", 10)

    def on_request(self, msg: String) -> None:
        self.pending = msg.data.strip().lower()

    def on_image(self, msg: Image) -> None:
        if self.pending is None:
            return
        rgb = image_to_rgb(msg)
        if rgb is None:
            return
        request, self.pending = self.pending, None
        hsv = to_hsv(rgb)
        total = float(msg.width * msg.height)
        min_area = float(self.get_parameter("min_area_frac").value) * total

        found = {}
        for color, ranges in ws.CUBE_HSV.items():
            if request and color != request:
                continue
            blobs = find_blobs(hsv, ranges, min_area, total)
            if not blobs:
                continue
            u, v, area = max(blobs, key=lambda b: b[2])
            found[color] = {
                "area_frac": area / total,
                "cx": 2.0 * u / msg.width - 1.0,
                "cy": 2.0 * v / msg.height - 1.0,
            }
        self.publisher.publish(String(data=json.dumps({"request": request, "colors": found})))


def main() -> None:
    rclpy.init()
    node = SimGripperDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
