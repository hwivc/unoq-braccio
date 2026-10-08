"""Teach the real arm to pick cubes by showing it, in the terminal.

    terminal 1:  ros2 launch unoq_braccio_bringup real.launch.py camera:=<camera>
    terminal 2:  ros2 run unoq_braccio_driver teach_pick --ros-args -p session:=desk

Each example: put one cube down (the camera reads it while the arm is out of
the way), drive the arm above it, down around it, close and lift, then say
whether it holds the cube. Every example refits the model, so from the
second one on the arm starts at its own guess and you only correct it. Every
``test_every`` examples (default 5) it offers a test: put a cube anywhere and
the arm tries on its own; ``s`` stops it, and a miss can be corrected on the
spot, which becomes a new example right where the model was weakest.

Everything is kept in ~/.ros/braccio_teach/<session>/ (see pick_learning).
Run the same session again to continue; use a new session name after moving
or zooming the camera. learned_pick_place.launch.py runs the result.
"""

import math
import os
import threading
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from unoq_braccio_driver import braccio_workspace as ws
from unoq_braccio_driver import pick_learning as pl
from unoq_braccio_driver.braccio_model import JOINT_LIMITS, JOINT_NAMES, START_POSE
from unoq_braccio_driver.braccio_protocol import parse_status
from unoq_braccio_driver.color_vision import image_to_rgb
from unoq_braccio_driver.real_setup import Terminal

SETUP_FILE = "~/.ros/braccio_setup.yaml"

# One joint per key pair, first key = larger servo value. Gripper: o opens, c closes.
JOINT_KEYS = {
    "j": (0, +1), "l": (0, -1),   # base: turn left / right
    "i": (1, +1), "k": (1, -1),   # shoulder: lean forward / back
    "y": (2, +1), "h": (2, -1),   # elbow: fold forward / back
    "t": (3, +1), "g": (3, -1),   # wrist: tip down / up
    "r": (4, +1), "f": (4, -1),   # wrist roll
    "c": (5, +1), "o": (5, -1),   # gripper: close / open
}
DRIVE_HELP = ("  j/l base  i/k shoulder  y/h elbow  t/g wrist  r/f roll  "
              "o/c gripper  O open  C grip  Tab 1/5 deg")


class Stopped(Exception):
    """The operator pressed s while the arm was moving on its own."""


class ArmLink(Node):
    """Arm commands, the arm's reported position and camera frames."""

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("setup_file", SETUP_FILE)
        self.declare_parameter("frames", 10)          # camera frames averaged per reading
        self.declare_parameter("speed_deg_s", 60.0)   # as serial_bridge; only for move timeouts
        self.declare_parameter("min_cube_frac", 0.001)
        self.declare_parameter("max_cube_frac", 0.06)
        ws.load_config(os.path.expanduser(str(self.get_parameter("setup_file").value)))
        self.colors = dict(ws.CUBE_HSV)
        self.gripper_open, self.gripper_closed = ws.GRIPPER_OPEN, ws.GRIPPER_CLOSED
        self.home = list(START_POSE[:5]) + [self.gripper_open]

        self.command = self.create_publisher(JointState, "/braccio/joint_command", 10)
        self.create_subscription(String, "/braccio/firmware_status", self.on_status, 10)
        self.create_subscription(Image, "/vision/overhead/image_raw", self.on_image, 2)
        self.pos = None          # servo degrees the firmware reports
        self.target = None       # servo degrees last commanded
        self.frame, self.frame_count = None, 0
        self.frame_event = threading.Event()

    def on_status(self, msg):
        status = parse_status(msg.data)
        if status is not None:
            self.pos = list(status["pos"])
            if self.target is None:
                self.target = list(self.pos)

    def on_image(self, msg):
        rgb = image_to_rgb(msg)
        if rgb is not None:
            self.frame, self.frame_count = rgb, self.frame_count + 1
            self.frame_event.set()

    def wait_ready(self, timeout=30.0):
        """Wait for the arm's position and a camera frame; False on timeout."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.pos is not None and self.frame is not None:
                return True
            time.sleep(0.1)
        return False

    # -- moving --------------------------------------------------------------

    def send(self, pose):
        pose = [max(JOINT_LIMITS[n].minimum, min(JOINT_LIMITS[n].maximum, int(round(v))))
                for n, v in zip(JOINT_NAMES, pose)]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in pose]
        self.command.publish(msg)
        self.target = pose
        return pose

    def freeze(self):
        """Stop where the arm is now (the firmware retargets mid-move)."""
        if self.pos is not None:
            self.send(self.pos)

    def move(self, pose, stop=None, settle=0.2):
        """Send ``pose`` and wait until the arm reports it is there.

        ``stop`` is polled while waiting; when it returns True the arm is
        frozen and Stopped is raised.
        """
        start = list(self.pos or self.target or pose)
        pose = self.send(pose)
        speed = float(self.get_parameter("speed_deg_s").value)
        end = time.monotonic() + max(abs(a - b) for a, b in zip(start, pose)) / speed * 1.5 + 2.0
        while time.monotonic() < end:
            if stop is not None and stop():
                self.freeze()
                raise Stopped()
            if self.pos is not None and max(abs(a - b) for a, b in zip(self.pos, pose)) <= 2:
                break
            time.sleep(0.05)
        time.sleep(settle)

    # -- camera ----------------------------------------------------------------

    def fresh_frames(self, count, timeout=6.0):
        frames, seen = [], self.frame_count
        end = time.monotonic() + timeout
        while len(frames) < count and time.monotonic() < end:
            self.frame_event.clear()
            self.frame_event.wait(0.5)
            if self.frame_count != seen:
                seen = self.frame_count
                frames.append(self.frame)
        return frames

    def detect(self, rgb):
        return pl.detect_cubes(rgb, self.colors,
                               float(self.get_parameter("min_cube_frac").value),
                               float(self.get_parameter("max_cube_frac").value))

    def read_cube(self):
        """(reading, None) for exactly one steady cube, else (None, reason)."""
        frames = self.fresh_frames(int(self.get_parameter("frames").value))
        return pl.summarise([self.detect(f) for f in frames])

    def read_cubes(self):
        """Every cube seen steadily over several frames (for the pick demo)."""
        frames = self.fresh_frames(int(self.get_parameter("frames").value))
        return pl.steady_cubes([self.detect(f) for f in frames])


def pose_text(pose):
    return ("base {} sh {} el {} wr {} roll {} grip {}".format(*pose)) if pose else "?"


class TeachPick(ArmLink):
    def __init__(self):
        super().__init__("teach_pick")
        self.declare_parameter("session", "default")
        self.declare_parameter("test_every", 5)
        self.declare_parameter("unsure_px", 60.0)      # further than this from every example: ask
        self.declare_parameter("check_camera", True)
        self.session = pl.Session(str(self.get_parameter("session").value))
        self.examples = self.session.examples()
        self.model = self.session.load_model()
        self.saved_at = self.model.info.get("examples", 0) if self.model else 0
        self.step = 1
        self.path_log = []

    # -- helpers -------------------------------------------------------------

    def refit(self, quiet=False):
        if not self.examples:
            self.model = None
            return
        self.model, scores = pl.train(self.examples)
        if not quiet:
            print("\n  Model refit on {} examples. Error on examples it did not see "
                  "(degrees, lower is better):".format(len(self.examples)))
            for name, err in scores.items():
                mark = "  <- using" if name == self.model.info["chosen"] else ""
                print(f"    {name:6s} {'-' if math.isinf(err) else f'{err:.1f}'}{mark}")

    def save_model(self):
        if self.model is not None and len(self.examples) != self.saved_at:
            name = self.session.save_model(self.model)
            self.saved_at = len(self.examples)
            print(f"  saved {name}")

    def model_text(self):
        if self.model is None:
            return "no model yet"
        err = self.model.info.get("error_deg")
        return "model {} ({}{})".format(
            self.model.info.get("name", "unsaved"), self.model.info.get("chosen"),
            f", ~{err:.1f} deg" if err is not None else "")

    def success_rate(self, last=10):
        tests = [t for t in self.session.tests() if t["result"] in ("ok", "fail")][-last:]
        if not tests:
            return ""
        ok = sum(t["result"] == "ok" for t in tests)
        return f"{ok}/{len(tests)} of the last {len(tests)} tests picked"

    def yes_no(self, term, prompt):
        print(f"\n{prompt} (y/n) ", end="", flush=True)
        while True:
            key = term.key(0.5)
            if key in ("y", "Y"):
                print("y")
                return True
            if key in ("n", "N", "\x1b"):
                print("n")
                return False

    def wait_enter(self, term, prompt, other=()):
        print(f"\n{prompt}", end="", flush=True)
        while True:
            key = term.key(0.5)
            if key in ("\n", "\r"):
                return "enter"
            if key in other:
                return key

    def drive(self, term, title, extra=()):
        """Jog with the keys until Enter ('enter'), Esc / x ('cancel') or an ``extra`` key."""
        print(f"\n{title}\n{DRIVE_HELP}\n  Enter = done   Esc/x = cancel this example")
        while True:
            print(f"\r  {pose_text(self.target)}  | step {self.step} deg   ", end="", flush=True)
            key = term.key(0.2)
            if key is None:
                continue
            if key in ("\n", "\r"):
                return "enter"
            if key in ("\x1b", "x"):
                return "cancel"
            if key in extra:
                return key
            if key == "\t":
                self.step = 5 if self.step == 1 else 1
                continue
            pose = list(self.target)
            if key in JOINT_KEYS:
                index, sign = JOINT_KEYS[key]
                pose[index] += sign * self.step
            elif key == "O":
                pose[5] = self.gripper_open
            elif key == "C":
                pose[5] = self.gripper_closed
            else:
                continue
            self.send(pose)
            self.path_log.append([round(time.time(), 2)] + list(self.target))

    def go_home(self, gripper=None):
        pose = list(self.home)
        if gripper is not None:
            pose[5] = gripper
        self.move(pose)

    def release_and_home(self, above, grab=None):
        """Put down whatever is held (at ``grab`` if given), open and go home."""
        if grab is not None:
            self.move(list(grab[:5]) + [self.target[5]])
        self.move(list(self.target[:5]) + [self.gripper_open], settle=0.5)
        self.move(list(above[:5]) + [self.gripper_open])
        self.go_home()

    # -- one example -----------------------------------------------------------

    def record_example(self, term, reading=None, kind="teach"):
        """PLACE -> ABOVE -> GRAB -> CLOSE -> CHECK. True if an example was saved."""
        self.go_home()
        if reading is None:
            reading, why = self.read_cube()
            if reading is None:
                print(f"\n  Cannot record: {why}.")
                return False
        if not self.examples and self.session.reference() is None:
            self.session.save_reference(self.frame)
            self.session.save_info({
                "created": time.strftime("%Y-%m-%d %H:%M:%S"),
                "image_size": [reading["width"], reading["height"]],
                "setup_file": str(self.get_parameter("setup_file").value),
            })
            print("\n  Saved the camera reference picture for this session.")
        print(f"\n  Cube: {reading['color']} at ({reading['u'] * reading['width']:.0f}, "
              f"{reading['v'] * reading['width']:.0f}) px, angle {reading['angle']:.0f} deg, "
              f"wobble {reading['jitter']:.1f} px")
        self.path_log = []

        guess = self.model.predict(reading) if self.model else None
        if guess:
            start = guess["above"]
        elif self.examples:
            start = list(self.examples[-1]["above"][:5]) + [self.gripper_open]
        else:
            start = list(self.home)
        self.move(start)
        if self.drive(term, "== ABOVE: put the open gripper above the cube ==") == "cancel":
            return self.cancel(start)
        above = list(self.target)

        if guess:  # the model's grab, with any roll you set while above
            self.move(guess["grab"][:4] + [above[4], above[5]])
        if self.drive(term, "== GRAB: lower the open fingers around the cube ==") == "cancel":
            return self.cancel(above)
        grab = list(self.target)

        if self.drive(term, "== CLOSE: close the gripper (C or c), then lift (e.g. k) ==") == "cancel":
            return self.cancel(above, grab)
        closed, lift = self.target[5], list(self.target)

        holds = self.yes_no(term, "== CHECK: is it holding the cube?")
        print("  putting the cube back down ...")
        self.release_and_home(above, grab)
        if not holds:
            print("  not saved.")
            return False
        example = {
            "id": time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:4],
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "kind": kind,
            **{k: reading[k] for k in ("u", "v", "angle", "area_frac", "color", "width", "height", "jitter")},
            "above": above, "grab": grab, "closed": closed, "lift": lift,
        }
        self.session.add_example(example)
        self.session.save_path(example["id"], self.path_log)
        self.examples.append(example)
        print(f"  saved example {len(self.examples)}.")
        every = int(self.get_parameter("test_every").value)
        due = len(self.examples) % every == 0
        self.refit(quiet=not due)
        if due:
            self.save_model()
            print(f"  {len(self.examples)} examples - time for a test.")
            self.test(term, prompt=True)
        return True

    def cancel(self, above, grab=None):
        print("\n  cancelled; opening and going home.")
        self.release_and_home(above, grab)
        return False

    # -- test ----------------------------------------------------------------------

    def test(self, term, prompt=False):
        if self.model is None:
            print("\n  Record at least one example first.")
            return
        self.go_home()
        if prompt and self.wait_enter(
                term, "TEST: put ONE cube anywhere, Enter = go, n = skip ", ("n", "N")) != "enter":
            return
        reading, why = self.read_cube()
        if reading is None:
            print(f"\n  Cannot test: {why}.")
            return
        guess = self.model.predict(reading)
        record = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "model": self.model.info.get("name"),
                  "chosen": self.model.info.get("chosen"), "reading": reading,
                  "guess": {k: guess[k] for k in ("above", "grab", "grip", "nearest_px")}}
        unsure = float(self.get_parameter("unsure_px").value)
        if guess["nearest_px"] > unsure:
            print(f"\n  I have not learned this area yet (closest example "
                  f"{guess['nearest_px']:.0f} px away).")
            self.session.log_test({**record, "result": "refused"})
            if self.yes_no(term, "  Teach it here now?"):
                self.record_example(term, reading)
            return

        print(f"\n  Going for the {reading['color']} cube.  s = STOP")
        stop = lambda: term.key(0.0) in ("s", "S")  # noqa: E731
        try:
            self.move(guess["above"], stop)
            self.move(guess["grab"], stop)
            self.move(guess["grip"], stop, settle=0.8)
            self.move(guess["lift"], stop)
            self.go_home(gripper=self.gripper_closed)
        except Stopped:
            print("\n  STOPPED.")
            self.session.log_test({**record, "result": "stopped"})
            self.wait_enter(term, "  Enter: open the gripper and go home ")
            self.release_and_home(guess["above"])
            return self.offer_correction(term)

        after = self.fresh_frames(int(self.get_parameter("frames").value))
        left = [pl.cube_near(self.detect(f), reading["u"], reading["v"]) for f in after]
        picked = sum(c is None for c in left) > len(left) / 2
        self.session.log_test({**record, "result": "ok" if picked else "fail"})
        if picked:
            print("  PICKED (the cube is gone from the table). Putting it back ...")
            self.move(guess["lift"])
            self.release_and_home(guess["above"], guess["grip"])
        else:
            print("  MISSED (the cube is still on the table).")
            self.go_home()
        print(f"  {self.success_rate()}")
        if not picked:
            self.offer_correction(term)

    def offer_correction(self, term):
        if self.yes_no(term, "  Correct it? (leave the cube where it is, then drive the arm onto it)"):
            self.record_example(term, kind="correction")

    # -- drop poses ------------------------------------------------------------------

    def teach_drops(self, term):
        drops = self.session.drops()
        print("\n== DROP POSES: for each colour, drive the closed gripper over where "
              "those cubes go ==")
        for color in self.colors:
            start = drops.get(color) or (list(self.home[:5]) + [self.gripper_closed])
            self.move(start)
            key = self.drive(term, f"  {color.upper()}: Enter = save, n = skip this colour", ("n",))
            if key == "enter":
                drops[color] = list(self.target)
                print(f"\n  saved the {color} drop pose.")
            elif key == "cancel":
                break
        self.session.save_drops(drops)
        self.go_home()

    # -- main loop -----------------------------------------------------------------

    def check_camera(self, term):
        reference = self.session.reference()
        if reference is None or not bool(self.get_parameter("check_camera").value):
            return True
        check = pl.compare_to_reference(self.frame, reference)
        if check is None:
            print("Camera check: the view does not look like this session's reference picture.")
        else:
            print("Camera check: shifted {:.1f} px, zoom {:+.1f}%, turned {:+.1f} deg".format(
                check["shift_px"], (check["scale"] - 1) * 100, check["rotation_deg"]))
        if not pl.camera_moved(check):
            return True
        print(f"  The camera seems to have moved since this session started. Put it back "
              f"to match {self.session.path('reference.jpg')}, or use a new session name.")
        return self.yes_no(term, "  Carry on anyway?")

    def run(self):
        print(f"Session '{self.session.name}' in {self.session.dir}")
        print("Waiting for the arm and the camera ...")
        if not self.wait_ready():
            print("No arm position or camera images. Is real.launch.py running?")
            return
        with Terminal() as term:
            if not self.check_camera(term):
                return
            if self.examples and self.model is None:
                self.refit(quiet=True)
            self.go_home()
            try:
                self.menu(term)
            finally:
                self.save_model()
                self.go_home()

    def menu(self, term):
        while True:
            drops = self.session.drops()
            print(f"\n[{len(self.examples)} examples | {self.model_text()} | "
                  f"drops: {', '.join(drops) or 'none'}] {self.success_rate()}")
            print("PLACE one cube.  Enter = record example   T = test   D = drop poses   "
                  "U = undo last example   Q = quit")
            last = 0.0
            while True:
                key = term.key(0.2)
                if key is None:
                    if time.monotonic() - last > 0.4 and self.frame is not None:
                        last = time.monotonic()
                        cubes = self.detect(self.frame)
                        status = ("no cube seen" if not cubes else
                                  "{} cube seen".format(cubes[0]["color"]) if len(cubes) == 1 else
                                  f"{len(cubes)} cubes seen - leave only ONE")
                        print(f"\r  {status:<48}", end="", flush=True)
                    continue
                key = key.lower()
                if key in ("\n", "\r"):
                    self.record_example(term)
                elif key == "t":
                    self.test(term)
                elif key == "d":
                    self.teach_drops(term)
                elif key == "u":
                    gone = self.session.remove_last_example()
                    if gone:
                        self.examples = self.session.examples()
                        self.refit(quiet=True)
                        print(f"\n  removed example {gone['id']}.")
                elif key == "q":
                    return
                else:
                    continue
                break


def spin_in_background(node):
    """Spin ``node`` on a thread; returns a function that stops it cleanly."""
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def stop():
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return stop


def main():
    rclpy.init()
    node = TeachPick()
    stop = spin_in_background(node)
    try:
        node.run()
    except KeyboardInterrupt:
        node.freeze()
        print("\nStopped. Examples recorded so far are saved.")
    finally:
        stop()


if __name__ == "__main__":
    main()
