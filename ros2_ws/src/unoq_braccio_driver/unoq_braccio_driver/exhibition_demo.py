"""Exhibition mode: shuffle cubes around the table for hours. Start real.launch.py first.

    ros2 run unoq_braccio_driver exhibition_demo --ros-args -p session:=desk
    (or the web dashboard's Show tab)

Each round the arm moves a few cubes: it picks one it knows (as
learned_pick_demo), puts it down on a free taught spot inside the base
window (see exhibition), checks with the camera, and goes on. Between rounds
it rests upright so the servos cool down, and every few rests it waves,
nods or dances. Settings (base window, rest, moves per round) are re-read
from ~/.ros/braccio_exhibition.yaml every round, so the web page can change
them while it runs.

It asks for help (state ATTENTION, the reason in /task/current) instead of
carrying on when: a cube went missing (it fell or rolled away; it carries on
by itself once the camera sees them all again, or with fewer cubes after
three more looks), it missed ``max_misses``
picks in a row, or no free spot is left. Stop with Ctrl+C: the arm freezes.

State (String) on /task/state and details (JSON) on /task/current, as
pick_place_demo, plus counters: moved, picks, misses, round, started.
"""

import json
import random
import time

import rclpy
from std_msgs.msg import String

from unoq_braccio_driver import exhibition as ex
from unoq_braccio_driver import pick_learning as pl
from unoq_braccio_driver.arm_tricks import TRICKS
from unoq_braccio_driver.teach_pick import ArmLink, spin_in_background


class ExhibitionDemo(ArmLink):
    def __init__(self):
        super().__init__("exhibition_demo")
        self.declare_parameter("session", "default")
        self.declare_parameter("model", "")            # "" = latest
        self.declare_parameter("unsure_px", 60.0)
        self.declare_parameter("settings_file", ex.SETTINGS_FILE)
        self.state_pub = self.create_publisher(String, "/task/state", 10)
        self.current_pub = self.create_publisher(String, "/task/current", 10)
        self.session = pl.Session(str(self.get_parameter("session").value))
        self.model = self.session.load_model(str(self.get_parameter("model").value))
        self.examples = self.session.examples()
        self.current = {"mode": "exhibition", "moved": 0, "picks": 0, "misses": 0, "round": 0,
                        "started": time.time(), "attention": ""}
        self.rng = random.Random()

    # -- reporting ---------------------------------------------------------------

    def publish_state(self, state, **details):
        self.current.update(details)
        self.current["state"] = state
        self.state_pub.publish(String(data=state))
        self.current_pub.publish(String(data=json.dumps(self.current, default=str)))
        shown = {k: v for k, v in details.items() if k not in ("pixel", "place_pixel")}
        self.get_logger().info(f"[{state}] {shown or ''}")

    def settings(self):
        return ex.load_settings(str(self.get_parameter("settings_file").value))

    # -- moves ------------------------------------------------------------------------

    def go_home(self, gripper=None):
        pose = list(self.home)
        if gripper is not None:
            pose[5] = gripper
        self.move(pose)

    def rest(self, seconds, label="Resting"):
        self.go_home()
        self.publish_state("REST", note=label, until=time.time() + seconds)
        time.sleep(seconds)

    def attract(self):
        trick = self.rng.choice(ex.ATTRACT_TRICKS)
        self.publish_state("ATTRACT", trick=trick)
        settings = self.settings()
        for pose, hold in TRICKS[trick]:
            pose = list(pose)
            if not ex.in_window(pose[0], settings):   # dance turns the base: keep it in the window
                lo = settings["base_min"] if settings["base_min"] is not None else pose[0]
                hi = settings["base_max"] if settings["base_max"] is not None else pose[0]
                pose[0] = max(lo, min(hi, pose[0]))
            self.send(pose)
            time.sleep(hold)
        self.go_home()

    def pick(self, cube, guess):
        """True when the camera no longer sees the cube where it was."""
        self.current["picks"] += 1
        self.publish_state("MOVE_ABOVE_CUBE", cube=cube["color"])
        self.move(guess["above"])
        self.publish_state("DESCEND")
        self.move(guess["grab"])
        self.publish_state("GRASP")
        self.move(guess["grip"], settle=0.8)
        self.publish_state("LIFT")
        self.move(guess["lift"])
        self.go_home(gripper=self.gripper_closed)
        self.publish_state("VERIFY_GRASP")
        frames = self.fresh_frames(int(self.get_parameter("frames").value))
        left = [pl.cube_near(self.detect(f), cube["u"], cube["v"]) for f in frames]
        return sum(c is None for c in left) > len(left) / 2

    def place(self, cube, spot):
        """Put the held cube down on ``spot``: the taught pick motion, opening instead of closing."""
        target = {"u": spot["u"], "v": spot["v"], "angle": cube["angle"]}
        guess = self.model.predict(target)
        closed = self.gripper_closed
        self.publish_state("MOVE_TO_BIN", bin=cube["color"])
        self.move([guess["above"][0]] + list(self.home[1:5]) + [closed])  # turn while upright
        self.move(guess["above"][:5] + [closed])
        self.publish_state("LOWER")
        self.move(guess["grab"][:5] + [closed], settle=0.4)
        self.publish_state("RELEASE")
        self.move(guess["grab"][:5] + [self.gripper_open], settle=0.5)
        self.publish_state("RETREAT")
        self.move(guess["above"][:5] + [self.gripper_open])
        self.go_home()

    # -- the show --------------------------------------------------------------------

    def run(self):
        if self.model is None or not self.examples:
            self.publish_state("FAILED", attention=f"Session '{self.session.name}' has no model yet: teach it first.")
            return
        if not self.wait_ready():
            self.publish_state("FAILED", attention="No arm position or camera images. Is the launch running?")
            return
        reference = self.session.reference()
        if reference is not None and pl.camera_moved(pl.compare_to_reference(self.frame, reference)):
            self.get_logger().warning("The camera view differs from this session's reference picture; "
                                      "picks may miss.")
        unsure = float(self.get_parameter("unsure_px").value)
        failures, recent, expected = {}, [], None
        misses_in_row = missing_checks = 0
        while True:
            settings = self.settings()
            if settings["max_minutes"] and time.time() - self.current["started"] > settings["max_minutes"] * 60:
                break
            self.current["round"] += 1
            for _ in range(settings["moves_per_round"]):
                self.publish_state("GO_HOME", attention="")
                self.go_home()
                self.publish_state("DETECTING")
                cubes = self.read_cubes()
                if expected is None or len(cubes) > expected:
                    expected = len(cubes)
                if len(cubes) < expected:
                    missing_checks += 1
                    if missing_checks >= 3:   # someone took it away for good: carry on with fewer
                        self.get_logger().warning(f"Carrying on with {len(cubes)} cubes.")
                        expected, missing_checks = len(cubes), 0
                    else:
                        self.attention(f"A cube is missing ({len(cubes)} of {expected} seen): it may "
                                       "have fallen. Please check the table.", settings)
                        break
                missing_checks = 0
                cube, guess = ex.choose_pick(cubes, self.model.predict, settings, unsure, failures, rng=self.rng)
                if cube is None:
                    self.attention(guess, settings)
                    break
                spots = ex.place_spots(self.examples, settings)
                spacing = 2.0 * ex.cube_size_px(cubes)
                spot = ex.choose_place(spots, cubes, cube, spacing, recent[-4:], rng=self.rng)
                if spot is None:
                    self.attention("No free spot to put a cube down: take a cube away, or teach more "
                                   "spots inside the base window.", settings)
                    break
                self.publish_state(
                    "TARGET_CONFIRMED", cube=cube["color"],
                    pixel=[round(cube["u"] * cube["width"]), round(cube["v"] * cube["width"])],
                    place_pixel=[round(spot["u"] * cube["width"]), round(spot["v"] * cube["width"])])
                if not self.pick(cube, guess):
                    failures[ex.spot_key(cube)] = failures.get(ex.spot_key(cube), 0) + 1
                    misses_in_row += 1
                    self.current["misses"] += 1
                    self.publish_state("VERIFY_FAILED", error="cube still on the table")
                    self.move(list(self.target[:5]) + [self.gripper_open], settle=0.5)
                    if misses_in_row >= settings["max_misses"]:
                        self.give_up(f"Missed {misses_in_row} picks in a row. Check the cubes are inside "
                                     "the taught area and the camera has not moved.", settings)
                        return
                    continue
                misses_in_row = 0
                self.place(cube, spot)
                recent.append((spot["u"], spot["v"]))
                self.publish_state("VERIFY_PLACEMENT")
                after = self.read_cubes()
                if pl.cube_near(after, spot["u"], spot["v"], radius=0.05) is not None:
                    self.current["moved"] += 1
                    self.publish_state("PLACED", place_pixel=None, pixel=None)
                elif len(after) < expected:
                    self.attention("The cube is not where it was put down and not anywhere else in view: "
                                   "it may have fallen. Please check the table.", settings)
                    break
                else:
                    self.current["moved"] += 1   # it moved, just not exactly onto the spot
                    self.publish_state("PLACED", note="landed a little off the spot", place_pixel=None, pixel=None)
            if self.current.get("attention"):
                continue
            if settings["attract"] and self.current["round"] % settings["attract_every"] == 0:
                self.attract()
            self.rest(settings["rest_s"])
        self.publish_state("GO_HOME")
        self.go_home()
        self.publish_state("COMPLETE")

    def attention(self, why, settings):
        """Something needs a person: say so, rest (with a fun move), then look again."""
        self.get_logger().warning(why)
        self.publish_state("ATTENTION", attention=why, place_pixel=None, pixel=None)
        self.go_home()
        if settings["attract"]:
            self.attract()
            self.publish_state("ATTENTION", attention=why)
        time.sleep(settings["rest_s"])

    def give_up(self, why, settings):
        """Too many misses: stop moving cubes, keep the arm safe and lively until stopped."""
        self.get_logger().error(why)
        self.publish_state("ATTENTION", attention=why, place_pixel=None, pixel=None)
        self.go_home()
        while True:
            time.sleep(settings["rest_s"])
            if settings["attract"]:
                self.attract()
                self.publish_state("ATTENTION", attention=why)


def main():
    rclpy.init()
    node = ExhibitionDemo()
    stop = spin_in_background(node)
    try:
        time.sleep(0.5)
        node.run()
    except KeyboardInterrupt:
        node.freeze()
    finally:
        stop()


if __name__ == "__main__":
    main()
