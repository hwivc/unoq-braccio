"""Calibrate the real overhead camera against the table (needs a screen).

    ros2 launch unoq_braccio_bringup real.launch.py camera:=http://192.168.1.192:8080/video
    ros2 run unoq_braccio_driver table_calibration

It reads unoq_braccio_bringup/config/real_workspace.yaml unless
``workspace_config`` points elsewhere.

Mark the ``calibration_points`` from the workspace config on the table (x
forward from the centre of the arm base, y to the arm's left, metres). Then,
for each point in turn, put ONE cube centred on the mark:

    SPACE        accept the cube the tool found (green cross)
    left click   use the clicked pixel instead (click the cube's centre)
    u            undo the last point
    q / Esc      quit

After the last point the homography is fitted, the error per point is
printed, and the result is saved to ``calibration_file``. The window then
switches to a check mode: move a cube anywhere and its (x, y) is shown live,
so you can confirm it with a ruler before letting the arm use it.

Redo this whenever the camera moves, zooms or changes resolution.
"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from unoq_braccio_driver import braccio_workspace as ws
from unoq_braccio_driver.color_vision import image_to_rgb, to_hsv
from unoq_braccio_driver.table_projection import (
    HomographyProjection,
    fit_homography,
    reprojection_errors,
    save_calibration,
)

WINDOW = "table calibration"
DEFAULT_FILE = "~/.ros/braccio_table_calibration.yaml"


def default_config() -> str:
    """The real-arm workspace file installed with unoq_braccio_bringup, or ""."""
    try:
        from ament_index_python.packages import get_package_share_directory

        path = os.path.join(
            get_package_share_directory("unoq_braccio_bringup"), "config", "real_workspace.yaml"
        )
    except Exception:  # package not built / not sourced
        return ""
    return path if os.path.exists(path) else ""


def largest_cube_blob(hsv):
    """Centre (u, v) of the biggest cube-coloured blob, or None.

    The centre of the blob's bounding box, not its centroid: the detector
    locates cubes by the centre of the box the model (or colour fallback)
    returns, and under a tilted camera the two differ by a few millimetres.
    Calibrating on the same point the detector measures cancels that out.

    Blobs touching the image border are skipped: a cube cut off by the edge of
    the frame has the wrong centre and would bend the whole calibration.
    """
    best, best_area = None, 0.0
    rows, cols = hsv.shape[:2]
    min_area = 0.0002 * rows * cols  # ignore specks
    for ranges in ws.CUBE_HSV.values():
        mask = None
        for low, high in ranges:
            part = cv2.inRange(hsv, np.array(low), np.array(high))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max(best_area, min_area):
                x, y, w, h = cv2.boundingRect(contour)
                if x <= 1 or y <= 1 or x + w >= cols - 1 or y + h >= rows - 1:
                    continue
                best, best_area = (x + w / 2.0, y + h / 2.0), area
    return best


class TableCalibration(Node):
    def __init__(self) -> None:
        super().__init__("table_calibration")
        self.declare_parameter("image_topic", "/vision/overhead/image_raw")
        self.declare_parameter("workspace_config", "")
        self.declare_parameter("calibration_file", DEFAULT_FILE)

        config = str(self.get_parameter("workspace_config").value) or default_config()
        if config:
            ws.load_config(os.path.expanduser(config))
            print(f"Workspace from {config}")
        self.output = os.path.expanduser(str(self.get_parameter("calibration_file").value))
        self.targets = list(ws.CALIBRATION_POINTS)
        self.pixels = []
        self.frame = None
        self.click = None
        self.result = None  # HomographyProjection once fitted

        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self.on_image, 2
        )

    def on_image(self, msg: Image) -> None:
        rgb = image_to_rgb(msg)
        if rgb is not None:
            self.frame = rgb

    def on_mouse(self, event, u, v, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click = (float(u), float(v))

    def add_point(self, pixel) -> None:
        index = len(self.pixels)
        self.pixels.append(pixel)
        x, y = self.targets[index]
        print(f"  point {index + 1}: table ({x:.3f}, {y:.3f}) m  <-  pixel ({pixel[0]:.1f}, {pixel[1]:.1f})")
        if len(self.pixels) == len(self.targets):
            self.fit()

    def fit(self) -> None:
        h = fit_homography(self.pixels, self.targets)
        errors = reprojection_errors(h, self.pixels, self.targets)
        size = (self.frame.shape[1], self.frame.shape[0])
        save_calibration(self.output, h, size, self.pixels, self.targets, errors)
        self.result = HomographyProjection(h, size)
        rms = 1000.0 * float(np.sqrt(np.mean(np.square(errors))))
        print(f"\nSaved {self.output}")
        print(f"Error per point (mm): {', '.join(f'{1000 * e:.1f}' for e in errors)}")
        print(f"RMS error: {rms:.1f} mm  "
              + ("(good)" if rms < 5 else "(high: check the marks and redo with u)"))
        print("Check mode: move a cube around and compare the shown x, y with a ruler.\n")

    def draw(self, image, detected) -> None:
        def text(line, row, colour=(255, 255, 255)):
            cv2.putText(image, line, (10, 22 + 22 * row), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(image, line, (10, 22 + 22 * row), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, colour, 1, cv2.LINE_AA)

        for i, (u, v) in enumerate(self.pixels):
            cv2.circle(image, (int(u), int(v)), 5, (255, 200, 0), 2)
            cv2.putText(image, str(i + 1), (int(u) + 7, int(v) - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1, cv2.LINE_AA)
        if detected is not None:
            u, v = int(detected[0]), int(detected[1])
            cv2.drawMarker(image, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 22, 2)

        if self.result is None:
            index = len(self.pixels)
            x, y = self.targets[index]
            text(f"Point {index + 1}/{len(self.targets)}: put ONE cube on x={x:.3f} y={y:.3f} m", 0)
            text("SPACE = use green cross, click = pick centre, u = undo, q = quit", 1)
            if detected is None:
                text("No cube seen. Cubes touching the image edge are ignored: "
                     "move the camera so every mark is well inside the view.", 2, (0, 200, 255))
        else:
            text("Saved. Check mode: move a cube, compare with a ruler. u = redo last, q = quit", 0)
            if detected is not None:
                x, y = self.result.to_table(*detected)
                text(f"cube at x={x:.3f} y={y:.3f} m", 1, (0, 255, 0))

    def run(self) -> None:
        cv2.namedWindow(WINDOW)
        cv2.setMouseCallback(WINDOW, self.on_mouse)
        print(f"Calibrating with {len(self.targets)} points; result -> {self.output}")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.03)
            if self.frame is None:
                waiting = np.zeros((240, 640, 3), np.uint8)
                cv2.putText(waiting, "Waiting for camera images...", (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
                cv2.imshow(WINDOW, waiting)
                if cv2.waitKey(30) & 0xFF in (ord("q"), 27):
                    break
                continue

            detected = largest_cube_blob(to_hsv(self.frame))
            image = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
            self.draw(image, detected)
            cv2.imshow(WINDOW, image)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            if key == ord("u") and self.pixels:
                self.pixels.pop()
                self.result = None
                print(f"  undid point {len(self.pixels) + 1}")
            elif self.result is None:
                if self.click is not None:
                    self.add_point(self.click)
                elif key == ord(" ") and detected is not None:
                    self.add_point(detected)
            self.click = None
        cv2.destroyAllWindows()


def main() -> None:
    rclpy.init()
    node = TableCalibration()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
