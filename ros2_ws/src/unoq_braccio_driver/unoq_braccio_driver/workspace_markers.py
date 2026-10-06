"""RViz markers for the workspace, the overhead camera's detections and task state.

    /workspace/markers   visualization_msgs/MarkerArray   (frame: world)

Shows the pick sector, the bin sectors (in the bins' own colours), the overhead
camera, every cube the overhead camera last reported, and the current task
state as text above the arm.
"""

import json
import os

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from unoq_braccio_driver import braccio_workspace as ws

CUBE_RGB = {"red": (1.0, 0.05, 0.05), "blue": (0.05, 0.2, 1.0), "yellow": (1.0, 0.85, 0.05)}
BIN_RGB = {"green": (0.1, 0.75, 0.2), "cyan": (0.0, 0.8, 0.85), "magenta": (0.85, 0.1, 0.8)}

STATE_RGB = {
    "COMPLETE": (0.2, 1.0, 0.3),
    "DETECTION_FAILED": (1.0, 0.2, 0.2),
    "TARGET_UNREACHABLE": (1.0, 0.2, 0.2),
    "VERIFY_FAILED": (1.0, 0.5, 0.1),
}


class WorkspaceMarkers(Node):
    def __init__(self) -> None:
        super().__init__("workspace_markers")
        # Defaults are the simulation; the real arm passes its workspace YAML
        # and turns off the simulated camera marker.
        self.declare_parameter("workspace_config", "")
        self.declare_parameter("show_camera", True)
        config = str(self.get_parameter("workspace_config").value)
        if config and os.path.exists(os.path.expanduser(config)):  # missing before real_setup
            ws.load_config(os.path.expanduser(config))
        self.cubes = []
        self.state = "IDLE"
        self.target = None
        self.publisher = self.create_publisher(MarkerArray, "/workspace/markers", 10)
        self.create_subscription(String, "/vision/cube_target", self.on_scan, 10)
        self.create_subscription(String, "/task/current", self.on_task, 10)
        self.create_timer(0.5, self.publish)

    def on_scan(self, msg: String) -> None:
        try:
            self.cubes = json.loads(msg.data).get("cubes", [])
        except json.JSONDecodeError:
            pass

    def on_task(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.state = data.get("state", self.state)
        if "x" in data and "y" in data:
            self.target = (data["x"], data["y"], data.get("cube", ""))

    def marker(self, marker_id, kind, x, y, z, sx, sy, sz, rgb, alpha=1.0, text=None):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "workspace"
        m.id = marker_id
        m.type = kind
        m.action = Marker.ADD
        # rclpy rejects ints in float fields, so everything is cast explicitly.
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color.r, m.color.g, m.color.b = (float(c) for c in rgb)
        m.color.a = float(alpha)
        if text is not None:
            m.text = text
        return m

    def publish(self) -> None:
        markers = MarkerArray()
        add = markers.markers.append

        pick = ws.PICK_SECTOR
        add(self.marker(1, Marker.CUBE, pick.x, pick.y, 0.001, pick.size_x, pick.size_y, 0.002,
                        (0.9, 0.9, 0.9), 0.35))
        add(self.marker(2, Marker.TEXT_VIEW_FACING, pick.x, pick.y, 0.05, 0, 0, 0.025,
                        (1, 1, 1), text="PICK"))

        for i, bin_ in enumerate(ws.BINS):
            rgb = BIN_RGB.get(bin_.name, (0.6, 0.6, 0.6))
            x, y = bin_.centre
            add(self.marker(10 + i, Marker.CUBE, x, y, bin_.height / 2, bin_.size, bin_.size,
                            bin_.height, rgb, 0.6))
            add(self.marker(20 + i, Marker.TEXT_VIEW_FACING, x, y, bin_.height + 0.04, 0, 0, 0.022,
                            (1, 1, 1), text=f"{bin_.cube_color.upper()} bin"))

        if bool(self.get_parameter("show_camera").value):
            cx, cy, cz = ws.CAMERA_XYZ
            add(self.marker(30, Marker.CUBE, cx, cy, cz, 0.04, 0.03, 0.02, (0.1, 0.1, 0.1)))
            # View axis down to the table.
            line = self.marker(31, Marker.LINE_LIST, 0, 0, 0, 0.002, 0, 0, (0.3, 0.6, 1.0), 0.6)
            line.points = [Point(x=float(cx), y=float(cy), z=float(cz)),
                           Point(x=float(cx), y=float(cy), z=0.0)]
            add(line)

        for i, cube in enumerate(self.cubes):
            rgb = CUBE_RGB.get(cube["color"], (0.5, 0.5, 0.5))
            add(self.marker(100 + i, Marker.CUBE, cube["x"], cube["y"], ws.CUBE_CENTRE_Z,
                            ws.CUBE_SIZE, ws.CUBE_SIZE, ws.CUBE_SIZE, rgb, 0.9))
            add(self.marker(120 + i, Marker.TEXT_VIEW_FACING, cube["x"], cube["y"], 0.06, 0, 0, 0.02,
                            (1, 1, 1), text=f"{cube['color']} [{cube['sector']}]"))
        for stale in range(len(self.cubes), 8):
            for base in (100, 120):
                gone = Marker()
                gone.header.frame_id = "world"
                gone.ns = "workspace"
                gone.id = base + stale
                gone.action = Marker.DELETE
                add(gone)

        rgb = STATE_RGB.get(self.state, (1.0, 1.0, 1.0))
        add(self.marker(200, Marker.TEXT_VIEW_FACING, 0.0, 0.0, 0.62, 0, 0, 0.05, rgb,
                        text=f"TASK: {self.state}"))
        if self.target is not None:
            add(self.marker(201, Marker.SPHERE, self.target[0], self.target[1], 0.06,
                            0.02, 0.02, 0.02, (1.0, 1.0, 1.0), 0.9))

        self.publisher.publish(markers)


def main() -> None:
    rclpy.init()
    node = WorkspaceMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
