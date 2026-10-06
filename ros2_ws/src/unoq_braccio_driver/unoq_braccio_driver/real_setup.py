"""Step-by-step set-up of the real arm, in the terminal.

    terminal 1:  ros2 launch unoq_braccio_bringup real.launch.py camera:=http://<phone-ip>:8080/video
    terminal 2:  ros2 run unoq_braccio_driver real_setup

1. Camera direction: + / - turn the arm until it points at the spot on the
   table right under the camera; Enter saves the angle.
2. Measurements: type the distance from the base centre to that spot, the
   camera height and the cube size (mm). The camera's x / y come from the
   angle and the distance.
3. Cube colours: one cube at a time under the camera; s saves the colour
   shown, n when done.
4. Drop points: for each colour, move the gripper over where those cubes
   should go (+ / - turn, w / s further / nearer); Enter saves.

Everything goes to ~/.ros/braccio_setup.yaml, which real.launch.py and
real_pick_place.launch.py read. The camera must look straight down with the
top of the picture pointing the way the arm faces.
"""

import math
import os
import select
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from unoq_braccio_driver.braccio_kinematics import solve_ik
from unoq_braccio_driver.braccio_model import JOINT_NAMES, START_POSE
from unoq_braccio_driver.color_vision import image_to_rgb, to_hsv
from unoq_braccio_driver.table_projection import focal_from_cube

SETUP_FILE = "~/.ros/braccio_setup.yaml"
POINT_GRIPPER = 105            # fingers closed to a point
POINT_REACH = 0.22             # how far out the arm points while aiming (m)
POINT_HEIGHT = 0.12            # fingertip height while aiming (m)
DROP_HOVER = 0.08              # fingertip height while choosing a drop point (m)
DROP_SIZE = 0.06               # a cube counts as "in" a drop point within this square (m)

# Cube pixels: saturated enough not to be white paper, grey table or shadow.
MIN_SATURATION = 60
MIN_VALUE = 50
HUE_NAMES = (("red", 0), ("orange", 14), ("yellow", 28), ("green", 60),
             ("cyan", 90), ("blue", 115), ("purple", 138), ("pink", 162))


# -- colour maths (pure, testable) ---------------------------------------------

def hue_distance(a, b):
    """Distance between two OpenCV hues (0-179, wrapping)."""
    d = abs(float(a) - float(b)) % 180.0
    return min(d, 180.0 - d)


def hue_name(hue):
    return min(HUE_NAMES, key=lambda named: hue_distance(hue, named[1]))[0]


def find_cube(hsv):
    """(x1, y1, x2, y2, area) of the biggest coloured blob, or None.
    Blobs touching the picture edge are skipped (cut-off cube)."""
    import cv2

    mask = ((hsv[:, :, 1] >= MIN_SATURATION) & (hsv[:, :, 2] >= MIN_VALUE)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rows, cols = hsv.shape[:2]
    best = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 0.0005 * rows * cols or (best and area <= best[4]):
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if x <= 1 or y <= 1 or x + w >= cols - 1 or y + h >= rows - 1:
            continue
        best = (x, y, x + w, y + h, area)
    return best


def measure_color(hsv, box):
    """(hue, hsv_ranges) of the cube in ``box``, or None.

    Measured on the raw camera pixels in the middle 60 % of the box, and only
    on coloured pixels, so the box drawn on screen, the background and
    shadows never count. Hue is a circular mean (red wraps at 0/179).
    """
    x1, y1, x2, y2 = box[:4]
    mx, my = (x2 - x1) * 0.2, (y2 - y1) * 0.2
    crop = hsv[int(y1 + my):int(y2 - my), int(x1 + mx):int(x2 - mx)].reshape(-1, 3).astype(float)
    cube = crop[(crop[:, 1] >= MIN_SATURATION) & (crop[:, 2] >= MIN_VALUE)]
    if len(cube) < 20:
        return None
    angles = cube[:, 0] * (math.pi / 90.0)
    hue = (math.degrees(math.atan2(np.sin(angles).mean(), np.cos(angles).mean())) / 2.0) % 180.0
    spread = float(np.percentile([hue_distance(h, hue) for h in cube[:, 0]], 90))
    half = max(6.0, min(18.0, spread + 4.0))
    s_low = int(max(MIN_SATURATION, np.percentile(cube[:, 1], 5) - 25))
    v_low = int(max(40, np.percentile(cube[:, 2], 5) - 40))
    lo, hi = hue - half, hue + half
    if lo < 0:
        spans = [(0, hi), (lo + 180, 179)]
    elif hi > 179:
        spans = [(lo, 179), (0, hi - 180)]
    else:
        spans = [(lo, hi)]
    ranges = [[[int(math.floor(a)), s_low, v_low], [int(math.ceil(b)), 255, 255]] for a, b in spans]
    return hue, half, ranges


# -- terminal keys -------------------------------------------------------------

class Terminal:
    """Single key presses without Enter (Linux terminal), plus normal input()."""

    def __init__(self):
        self.fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self.saved = None

    def __enter__(self):
        if self.fd is not None:
            import termios
            import tty

            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_):
        self.restore()

    def restore(self):
        if self.saved is not None:
            import termios

            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def key(self, timeout=0.2):
        """One key, or None after ``timeout`` seconds."""
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None

    def ask(self, prompt, default):
        """Ask for a number (normal line input); Enter keeps the default."""
        self.restore()
        try:
            while True:
                answer = input(f"{prompt} [{default:g}]: ").strip().replace(",", ".")
                if not answer:
                    return float(default)
                try:
                    return float(answer)
                except ValueError:
                    print("  please type a number")
        finally:
            if self.fd is not None:
                import tty

                tty.setcbreak(self.fd)


# -- the set-up node -------------------------------------------------------------

class RealSetup(Node):
    def __init__(self):
        super().__init__("real_setup")
        self.declare_parameter("setup_file", SETUP_FILE)
        self.path = os.path.expanduser(str(self.get_parameter("setup_file").value))
        self.setup = {}
        if os.path.exists(self.path):
            with open(self.path, encoding="utf-8") as handle:
                self.setup = yaml.safe_load(handle) or {}
        self.frame = None
        self.command = self.create_publisher(JointState, "/braccio/joint_command", 10)
        self.create_subscription(Image, "/vision/overhead/image_raw", self.on_image, 2)

    def on_image(self, msg):
        rgb = image_to_rgb(msg)
        if rgb is not None:
            self.frame = rgb

    def send(self, pose):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in pose]
        self.command.publish(msg)

    def point(self, base_deg, reach, height):
        """Lean the arm out at ``base_deg`` (servo degrees); False if unreachable."""
        angle = math.radians(base_deg - 90.0)
        pose = solve_ik(reach * math.cos(angle), reach * math.sin(angle), height, POINT_GRIPPER, 90)
        if pose is None:
            return False
        self.send(pose)
        return True

    def save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, "w", encoding="utf-8") as handle:
            handle.write("# Written by real_setup. Re-run it rather than editing by hand:\n"
                         "#   ros2 run unoq_braccio_driver real_setup\n")
            yaml.safe_dump(self.setup, handle, sort_keys=False)

    # -- steps --------------------------------------------------------------------

    def step_camera(self, term):
        print("\n== STEP 1/4: CAMERA DIRECTION ==")
        print("The arm will lean out. Keep hands clear. Press Enter to start.")
        while term.key(1.0) not in ("\n", "\r"):
            pass
        base = float(self.setup.get("camera_base_deg", 90.0))
        self.point(base, POINT_REACH, POINT_HEIGHT)
        print("Turn the arm until it points at the spot on the table right UNDER the camera.")
        print("  + / -  turn 1 degree     ] / [  turn 5 degrees     Enter  save")
        while True:
            key = term.key()
            step = {"+": 1, "=": 1, "-": -1, "_": -1, "]": 5, "[": -5}.get(key)
            if step:
                new = max(0.0, min(180.0, base + step))
                if self.point(new, POINT_REACH, POINT_HEIGHT):
                    base = new
                print(f"\r  base angle {base:.0f}   ", end="", flush=True)
            elif key in ("\n", "\r"):
                break
        print(f"\nSaved camera direction: base {base:.0f} degrees")
        self.send(START_POSE)

        print("\n== STEP 2/4: MEASUREMENTS (millimetres) ==")
        distance = term.ask("Distance from the base centre to the spot under the camera",
                            self.setup.get("camera_distance_mm", 200))
        height = term.ask("Camera height: lens straight down to the table",
                          self.setup.get("camera_height_mm", 600))
        cube = term.ask("Cube size (edge length)", self.setup.get("cube_size_mm", 30))
        angle = math.radians(base - 90.0)
        self.setup.update({
            "camera_base_deg": round(base, 1),
            "camera_distance_mm": round(distance, 1),
            "camera_height_mm": round(height, 1),
            "camera_x_mm": round(distance * math.cos(angle), 1),
            "camera_y_mm": round(distance * math.sin(angle), 1) + 0.0,
            "cube_size_mm": round(cube, 1),
        })
        self.save()
        print(f"Camera at x {self.setup['camera_x_mm']:.0f} mm, y {self.setup['camera_y_mm']:.0f} mm, "
              f"{height:.0f} mm up. Saved.")

    def step_colors(self, term):
        import cv2  # noqa: F401  (OpenCV is needed by find_cube)

        print("\n== STEP 3/4: CUBE COLOURS ==")
        print("Put ONE cube on the table right under the camera.")
        print("  s  save the colour shown     u  undo last     n  done")
        colors = []
        fx_samples = []
        last_print = 0.0
        current = None
        while True:
            frame = self.frame
            if frame is None:
                if time.monotonic() - last_print > 2.0:
                    print("  waiting for camera images (is real.launch.py running?)")
                    last_print = time.monotonic()
            else:
                hsv = to_hsv(frame)
                box = find_cube(hsv)
                measured = measure_color(hsv, box) if box else None
                current = None
                if measured:
                    hue, half, ranges = measured
                    name = hue_name(hue)
                    clash = [c["name"] for c in colors
                             if hue_distance(c["hue"], hue) < c["half"] + half + 2]
                    current = (name, hue, half, ranges, box, frame.shape, clash)
                    status = f"seeing {name} (hue {hue:.0f})" + (f" - too close to {clash[0]}" if clash else "")
                else:
                    status = "no cube seen"
                print(f"\r  {status:<50}", end="", flush=True)

            key = term.key()
            if key == "s" and current and not current[6]:
                name, hue, half, ranges, box, shape, _ = current
                unique, n = name, 2
                while unique in [c["name"] for c in colors]:
                    unique, n = f"{name}{n}", n + 1
                colors.append({"name": unique, "hue": hue, "half": half, "ranges": ranges})
                # The cube's known size also measures the camera's zoom.
                rows, cols = shape[:2]
                cu, cv_ = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
                if abs(cu - cols / 2) < cols / 4 and abs(cv_ - rows / 2) < rows / 4:
                    fx_samples.append(focal_from_cube(
                        math.sqrt(box[4]), self.setup["camera_height_mm"] / 1000.0,
                        self.setup["cube_size_mm"] / 1000.0))
                print(f"\n  saved colour {unique}. Next cube, or n when done.")
            elif key == "u" and colors:
                print(f"\n  removed {colors.pop()['name']}")
            elif key == "n":
                if not colors:
                    print("\n  save at least one colour first (s)")
                    continue
                break
        self.setup["cube_hsv"] = {c["name"]: c["ranges"] for c in colors}
        if fx_samples:
            self.setup["camera_fx_px"] = round(sorted(fx_samples)[len(fx_samples) // 2], 1)
        self.save()
        print(f"\nSaved colours: {', '.join(c['name'] for c in colors)}")

    def step_drops(self, term):
        names = list(self.setup.get("cube_hsv", {}))
        cube = self.setup["cube_size_mm"] / 1000.0
        print("\n== STEP 4/4: DROP POINTS ==")
        print("For each colour, move the gripper over where those cubes should go.")
        print("  + / -  turn 1 degree   ] / [  5 degrees   w / s  further / nearer   Enter  save")
        bins = []
        for i, name in enumerate(names):
            base, reach = 90.0 + 30.0 + 20.0 * i, 0.25   # start on the arm's left
            print(f"\n  Drop point for {name.upper()} cubes:")
            while not self.point(base, reach, DROP_HOVER) and base > 0:
                base -= 5.0
            while True:
                key = term.key()
                turn = {"+": 1, "=": 1, "-": -1, "_": -1, "]": 5, "[": -5}.get(key, 0)
                out = {"w": 0.005, "s": -0.005}.get(key, 0.0)
                if turn or out:
                    new_base = max(0.0, min(180.0, base + turn))
                    new_reach = reach + out
                    angle = math.radians(new_base - 90.0)
                    x, y = new_reach * math.cos(angle), new_reach * math.sin(angle)
                    if (solve_ik(x, y, cube / 2 + 0.02, 95) is not None
                            and self.point(new_base, new_reach, DROP_HOVER)):
                        base, reach = new_base, new_reach
                    print(f"\r  base {base:.0f} deg, {reach * 100:.1f} cm out   ", end="", flush=True)
                elif key in ("\n", "\r"):
                    angle = math.radians(base - 90.0)
                    x, y = reach * math.cos(angle), reach * math.sin(angle)
                    # Drop points must not overlap, or a cube on one would be
                    # read as being on the other.
                    near = [b["cube_color"] for b in bins
                            if math.hypot(b["x"] - x, b["y"] - y) < DROP_SIZE + 0.02]
                    if not near:
                        break
                    print(f"\n  too close to the {near[0]} drop point (keep them "
                          f"{(DROP_SIZE + 0.02) * 100:.0f} cm apart); move further away")
            bins.append({"name": f"drop_{name}", "cube_color": name, "x": round(x, 4),
                         "y": round(y, 4) + 0.0, "size": DROP_SIZE, "height": 0.0})
            print(f"\n  saved: {name} cubes go to x {x * 100:.1f} cm, y {y * 100:.1f} cm")
        self.send(START_POSE)
        self.setup["bins"] = bins
        # Pick any reachable cube that is not on a drop point.
        self.setup["pick_area"] = {"x": 0.17, "y": 0.0, "size_x": 0.46, "size_y": 0.80}
        self.save()

    def run(self):
        with Terminal() as term:
            try:
                self.step_camera(term)
                self.step_colors(term)
                self.step_drops(term)
            finally:
                self.send(START_POSE)
        print(f"\nAll saved to {self.path}.")
        print("Restart real.launch.py, then: ros2 launch unoq_braccio_bringup real_pick_place.launch.py")


def main():
    rclpy.init()
    node = RealSetup()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        time.sleep(0.5)  # let the publisher connect
        node.run()
    except KeyboardInterrupt:
        print("\nStopped; steps finished so far are saved.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
