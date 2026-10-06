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
from unoq_braccio_driver import braccio_workspace as ws
from unoq_braccio_driver.table_projection import (
    PinholeDownProjection,
    fit_homography,
    focal_from_cube,
    reprojection_errors,
)

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


# -- smooth nudging --------------------------------------------------------------

class Jog:
    """Moves the fingertip by base angle and reach at a fixed height, with
    every joint changing smoothly and each key press moving about the same.

    solve_ik picks the steepest tool pitch that fits each target, so the
    pitch jumps by 5 degrees every few millimetres of reach: the shoulder
    swings back while you asked to go further, and one press moves 1-9 mm.
    No single pitch covers the whole reach either (each spans ~7 cm). So the
    pitch here is the middle of the range that fits at this reach: it slides
    smoothly as you reach out and stays clear of the joint limits. The
    servos only take whole degrees, so instead of rounding each joint on its
    own (errors add up), the whole-degree combination whose fingertip lands
    closest to the target is sent.
    """

    def __init__(self, send, height, gripper):
        self.send, self.height, self.gripper = send, height, gripper
        self.joints = None   # shoulder, elbow, wrist_vertical (floats)

    def move(self, base, reach):
        from unoq_braccio_driver.braccio_kinematics import forward_kinematics, planar_ik

        fits = [p for p in range(-90, -9) if planar_ik(reach, self.height, p) is not None]
        if not fits:
            return False
        middle = (fits[0] + fits[-1]) / 2.0
        joints = planar_ik(reach, self.height, middle, near=self.joints)
        if joints is None:  # the fitting range has a gap: use the closest that fits
            joints = planar_ik(reach, self.height, min(fits, key=lambda p: abs(p - middle)),
                               near=self.joints)
        self.joints = joints
        base = int(round(base))
        angle = math.radians(base - 90.0)
        target = (reach * math.cos(angle), reach * math.sin(angle), self.height)
        best = None
        for ds in (math.floor, math.ceil):
            for de in (math.floor, math.ceil):
                for dw in (math.floor, math.ceil):
                    servos = [base, ds(joints[0]), de(joints[1]), dw(joints[2]), 90]
                    miss = math.dist(forward_kinematics(servos), target)
                    if best is None or miss < best[0]:
                        best = (miss, servos)
        self.send(best[1] + [self.gripper])
        return True


# -- the set-up node -------------------------------------------------------------

class RealSetup(Node):
    def __init__(self):
        super().__init__("real_setup")
        self.declare_parameter("setup_file", SETUP_FILE)
        # Which steps to run: camera, colors, drops, touch (comma separated).
        self.declare_parameter("steps", "camera,colors,drops,touch")
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
        print("\n== STEP 1/5: CAMERA DIRECTION ==")
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

        print("\n== STEP 2/5: MEASUREMENTS (millimetres) ==")
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

        print("\n== STEP 3/5: CUBE COLOURS ==")
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
        print("\n== STEP 4/5: DROP POINTS ==")
        print("For each colour, move the gripper over where those cubes should go.")
        print("  + / -  turn 1 degree   ] / [  5 degrees   w / s  further / nearer 5 mm   "
              "W / S  2 mm   Enter  save")
        bins = []
        for i, name in enumerate(names):
            base, reach = 90.0 + 30.0 + 20.0 * i, 0.25   # start on the arm's left
            print(f"\n  Drop point for {name.upper()} cubes:")
            jog = Jog(self.send, DROP_HOVER, POINT_GRIPPER)
            while not jog.move(base, reach) and base > 0:
                base -= 5.0
            while True:
                key = term.key()
                turn = {"+": 1, "=": 1, "-": -1, "_": -1, "]": 5, "[": -5}.get(key, 0)
                out = {"w": 0.005, "s": -0.005, "W": 0.002, "S": -0.002}.get(key, 0.0)
                if turn or out:
                    new_base = max(0.0, min(180.0, base + turn))
                    new_reach = reach + out
                    angle = math.radians(new_base - 90.0)
                    x, y = new_reach * math.cos(angle), new_reach * math.sin(angle)
                    if (solve_ik(x, y, cube / 2 + 0.02, 95) is not None
                            and jog.move(new_base, new_reach)):
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

    def touch_detector(self):
        """The same cube finder the running detector uses (Edge Impulse model
        if available, else the learned colours), so both measure the same point."""
        from unoq_braccio_driver.cube_model_backend import create_cube_detector

        cube = self.setup["cube_size_mm"] / 1000.0
        height = self.setup.get("camera_height_mm", 600) / 1000.0
        fx = float(self.setup.get("camera_fx_px", 0) or 500.0)
        ws.apply_config({"cube_size": cube, "cube_hsv": self.setup["cube_hsv"]})
        return create_cube_detector("edge_impulse", "", 0.3, 0.45, cube, fx, height - cube / 2)

    def first_guess(self, u, v, shape):
        """Table (x, y) of pixel (u, v) from the measured camera (steps 1-2)."""
        s = self.setup
        if not all(k in s for k in ("camera_x_mm", "camera_y_mm", "camera_height_mm")):
            return 0.22, 0.0
        fx = float(s.get("camera_fx_px", 0) or 500.0)
        camera = PinholeDownProjection(fx, fx, shape[1] / 2.0, shape[0] / 2.0,
                                       s["camera_x_mm"] / 1000.0, s["camera_y_mm"] / 1000.0,
                                       s["camera_height_mm"] / 1000.0)
        return camera.to_table(u, v, s["cube_size_mm"] / 2000.0)

    def step_touch(self, term):
        print("\n== STEP 5/5: TOUCH CALIBRATION ==")
        print("Put ONE cube anywhere on the table, then press Enter: the arm moves over it.")
        print("Nudge until the closed fingertips are centred right over the cube, Enter saves.")
        print("  + / -  turn 1 degree   ] / [  5 degrees   w / s  further / nearer 5 mm   W / S  2 mm")
        print("  u  undo last point     d  done (at least 4; 6-8 spread out is best)")
        detector = self.touch_detector()
        cube = self.setup["cube_size_mm"] / 1000.0
        hover = cube + 0.02            # grasp point 2 cm above the cube's top
        pixels, targets = [], []
        aiming = None                  # (u, v, base, reach) while nudging
        shape = None
        last_wait = 0.0
        while True:
            if aiming is None:
                frame = self.frame
                if frame is None:
                    if time.monotonic() - last_wait > 2.0:
                        print("\n  waiting for camera images (is real.launch.py running?)")
                        last_wait = time.monotonic()
                    seen = []
                else:
                    shape = frame.shape
                    seen = detector.find_cubes(frame)
                    status = ("no cube seen" if not seen else
                              "cube seen - Enter to move the arm over it" if len(seen) == 1 else
                              f"{len(seen)} cubes seen - leave only ONE on the table")
                    print(f"\r  [{len(pixels)} points] {status:<48}", end="", flush=True)
            key = term.key()
            if aiming is None:
                if key in ("\n", "\r") and shape is not None and len(seen) == 1:
                    x1, y1, x2, y2 = seen[0][:4]
                    u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                    x, y = self.first_guess(u, v, shape)
                    base = 90.0 + math.degrees(math.atan2(y, x))
                    reach = math.hypot(x, y)
                    jog = Jog(self.send, hover, POINT_GRIPPER)
                    if not jog.move(base, reach):
                        base, reach = 90.0, 0.22
                        jog.move(base, reach)
                    aiming = (u, v, base, reach)
                    print(f"\n  arm over the guess ({x * 100:.1f}, {y * 100:.1f}) cm - nudge it, Enter saves")
                elif key == "u" and pixels:
                    pixels.pop()
                    targets.pop()
                    print(f"\n  removed the last point ({len(pixels)} left)")
                elif key == "d":
                    if len(pixels) < 4:
                        print(f"\n  need at least 4 points (have {len(pixels)})")
                        continue
                    break
            else:
                u, v, base, reach = aiming
                turn = {"+": 1, "=": 1, "-": -1, "_": -1, "]": 5, "[": -5}.get(key, 0)
                out = {"w": 0.005, "s": -0.005, "W": 0.002, "S": -0.002}.get(key, 0.0)
                if turn or out:
                    new_base, new_reach = max(0.0, min(180.0, base + turn)), reach + out
                    if jog.move(new_base, new_reach):
                        aiming = (u, v, new_base, new_reach)
                    print(f"\r  base {aiming[2]:.0f} deg, {aiming[3] * 100:.1f} cm out   ",
                          end="", flush=True)
                elif key in ("\n", "\r"):
                    angle = math.radians(base - 90.0)
                    pixels.append((u, v))
                    targets.append((reach * math.cos(angle), reach * math.sin(angle)))
                    self.point(base, reach, hover + 0.06)   # lift clear of the cube
                    aiming = None
                    print(f"\n  saved point {len(pixels)}. Move the cube somewhere else "
                          "(or d when done).")

        h = fit_homography(pixels, targets)
        errors = reprojection_errors(h, pixels, targets)
        print("\nFit error per point (mm): " + ", ".join(f"{e * 1000:.1f}" for e in errors))
        if len(pixels) >= 5:
            # Leave-one-out: predict each point from the others; a big number
            # here means that point was nudged wrong (u, then redo it).
            checks = []
            for i in range(len(pixels)):
                rest = [j for j in range(len(pixels)) if j != i]
                try:
                    hi = fit_homography([pixels[j] for j in rest], [targets[j] for j in rest])
                    checks.append(reprojection_errors(hi, [pixels[i]], [targets[i]])[0])
                except ValueError:
                    checks.append(float("nan"))
            print("Check, each point from the others (mm): "
                  + ", ".join(f"{e * 1000:.1f}" for e in checks))
            print("Under ~5-10 mm is good; a much bigger one is a point to redo.")
        rms = 1000.0 * math.sqrt(sum(e * e for e in errors) / len(errors))
        self.setup.update({
            "homography": [[float(c) for c in row] for row in h],
            "image_width": int(shape[1]),
            "image_height": int(shape[0]),
            "touch_rms_mm": round(rms, 2),
            "touch_points": [{"pixel": [round(u, 1), round(v, 1)], "arm": [round(x, 4), round(y, 4)]}
                             for (u, v), (x, y) in zip(pixels, targets)],
        })
        self.save()
        self.send(START_POSE)
        print(f"Touch calibration saved ({len(pixels)} points). The detector uses it after a restart.")

    def run(self):
        wanted = [s.strip().lower() for s in str(self.get_parameter("steps").value).split(",")]
        steps = [(name, getattr(self, f"step_{name}"))
                 for name in ("camera", "colors", "drops", "touch") if name in wanted]
        with Terminal() as term:
            try:
                for name, step in steps:
                    if name != "camera" and "cube_size_mm" not in self.setup:
                        print("Run the camera step first (it asks for the cube size).")
                        return
                    if name in ("drops", "touch") and not self.setup.get("cube_hsv"):
                        print("Run the colors step first.")
                        return
                    step(term)
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
