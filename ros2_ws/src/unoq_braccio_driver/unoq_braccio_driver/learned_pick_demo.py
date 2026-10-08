"""Pick and place with a model taught by teach_pick. Start real.launch.py first.

    ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=desk
    ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=desk model:=model_0010
    ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=desk colors:=red,blue

Looks at the table with the arm out of the way, picks the cube the model
knows best, checks with the camera that it left the table, drops it, and
repeats until no cube it can handle is left. ``colors`` limits it to some
colours (others are left alone); empty = every colour. ``drop``:

    auto   the drop pose taught for the colour (teach_pick D), else side
    taught only colours with a taught drop pose are picked
    side   turn the base to ``drop_base`` (0 = the far end of the l key),
           reach out as the cube was picked, and let go

Cubes far from every example are left alone. Stop with Ctrl+C: the arm
freezes where it is.

State (String) on /task/state and details (JSON) on /task/current, as
pick_place_demo.
"""

import json
import time

import rclpy
from std_msgs.msg import String

from unoq_braccio_driver import pick_learning as pl
from unoq_braccio_driver.pick_place_demo import State
from unoq_braccio_driver.teach_pick import ArmLink, spin_in_background


class LearnedPickDemo(ArmLink):
    def __init__(self):
        super().__init__("learned_pick_demo")
        self.declare_parameter("session", "default")
        self.declare_parameter("model", "")            # "" = latest, or e.g. model_0010
        self.declare_parameter("unsure_px", 60.0)
        self.declare_parameter("attempts", 2)          # tries per cube before leaving it
        self.declare_parameter("max_picks", 20)
        self.declare_parameter("colors", "")            # e.g. "red,blue"; "" = every colour
        self.declare_parameter("drop", "auto")          # auto | taught | side
        self.declare_parameter("drop_base", 0)          # base angle for side drops (l key end)
        self.state_pub = self.create_publisher(String, "/task/state", 10)
        self.current_pub = self.create_publisher(String, "/task/current", 10)
        self.current = {}
        self.session = pl.Session(str(self.get_parameter("session").value))
        self.model = self.session.load_model(str(self.get_parameter("model").value))
        self.drops = self.session.drops()

    def publish_state(self, state, **details):
        self.current.update(details)
        self.current["state"] = state.value
        self.state_pub.publish(String(data=state.value))
        self.current_pub.publish(String(data=json.dumps(self.current, default=str)))
        self.get_logger().info(f"[{state.value}] {details or ''}")

    def pick(self, cube, guess):
        """One attempt: True when the camera no longer sees the cube on the table."""
        self.publish_state(State.MOVE_ABOVE_CUBE, cube=cube["color"])
        self.move(guess["above"])
        self.publish_state(State.DESCEND)
        self.move(guess["grab"])
        self.publish_state(State.GRASP)
        self.move(guess["grip"], settle=0.8)
        self.publish_state(State.LIFT)
        self.move(guess["lift"])
        self.go_home(gripper=self.gripper_closed)
        self.publish_state(State.VERIFY_GRASP)
        frames = self.fresh_frames(int(self.get_parameter("frames").value))
        left = [pl.cube_near(self.detect(f), cube["u"], cube["v"]) for f in frames]
        return sum(c is None for c in left) > len(left) / 2

    def drop_pose(self, color, guess):
        """Where this cube is let go (gripper closed), or None to leave it."""
        mode = str(self.get_parameter("drop").value).lower()
        if mode != "side" and color in self.drops:
            return list(self.drops[color][:5]) + [self.gripper_closed]
        if mode == "taught":
            return None
        return [int(self.get_parameter("drop_base").value)] + list(guess["above"][1:5]) + [self.gripper_closed]

    def drop(self, color, guess):
        pose = self.drop_pose(color, guess)
        self.publish_state(State.MOVE_TO_BIN, bin=color, drop=pose)
        # Turn while standing up, then reach out, so nothing on the table is swept.
        self.move([pose[0]] + list(self.home[1:5]) + [self.gripper_closed])
        self.move(pose)
        self.publish_state(State.RELEASE)
        self.move(pose[:5] + [self.gripper_open], settle=0.5)
        self.move([pose[0]] + list(self.home[1:5]) + [self.gripper_open])

    def go_home(self, gripper=None):
        pose = list(self.home)
        if gripper is not None:
            pose[5] = gripper
        self.move(pose)

    def run(self):
        wanted = [c.strip().lower() for c in str(self.get_parameter("colors").value).split(",")
                  if c.strip()]
        unknown = [c for c in wanted if c not in self.colors]
        if unknown:
            self.get_logger().error(
                f"Unknown colour(s) {', '.join(unknown)}; known: {', '.join(self.colors)}. "
                f"Add one with: ros2 run unoq_braccio_driver real_setup --ros-args -p steps:=colors")
            return
        if self.model is None:
            self.get_logger().error(
                f"No model in {self.session.dir}/models. Teach one first: "
                f"ros2 run unoq_braccio_driver teach_pick --ros-args -p session:={self.session.name}")
            return
        self.get_logger().info(
            f"Session '{self.session.name}', {self.model.info.get('name')} "
            f"({self.model.info.get('chosen')}, {self.model.info.get('examples')} examples); "
            f"drop poses: {', '.join(self.drops) or 'none'}; drop mode "
            f"{self.get_parameter('drop').value}; colours: {', '.join(wanted) or 'all'}")
        if not self.wait_ready():
            self.get_logger().error("No arm position or camera images. Is real.launch.py running?")
            return
        reference = self.session.reference()
        if reference is not None and pl.camera_moved(pl.compare_to_reference(self.frame, reference)):
            self.get_logger().warning("The camera view differs from this session's reference "
                                      "picture; picks may miss.")

        unsure = float(self.get_parameter("unsure_px").value)
        failures, placed = {}, []
        start = None  # cubes on the table at the start; dropped ones are never re-picked
        for _ in range(int(self.get_parameter("max_picks").value)):
            self.publish_state(State.GO_HOME)
            self.go_home()
            self.publish_state(State.DETECTING)
            cubes = self.read_cubes()
            if start is None:
                start = cubes
            options = []
            for cube in cubes:
                if pl.cube_near(start, cube["u"], cube["v"]) is None:
                    continue
                guess = self.model.predict(cube)
                key = (round(cube["u"], 2), round(cube["v"], 2))
                if wanted and cube["color"] not in wanted:
                    continue
                if (guess["nearest_px"] <= unsure and self.drop_pose(cube["color"], guess) is not None
                        and failures.get(key, 0) < int(self.get_parameter("attempts").value)):
                    options.append((guess["nearest_px"], key, cube, guess))
            if not options:
                break
            _, key, cube, guess = min(options, key=lambda o: o[0])  # best-known spot first
            self.publish_state(State.TARGET_CONFIRMED, cube=cube["color"],
                               pixel=[round(cube["u"] * cube["width"]), round(cube["v"] * cube["width"])],
                               angle=round(cube["angle"]))
            if not self.pick(cube, guess):
                failures[key] = failures.get(key, 0) + 1
                self.publish_state(State.VERIFY_FAILED, error="cube still on the table")
                self.move(list(self.target[:5]) + [self.gripper_open], settle=0.5)
                continue
            self.drop(cube["color"], guess)
            placed.append(cube["color"])
        self.publish_state(State.GO_HOME)
        self.go_home()
        left = [c["color"] for c in self.read_cubes()
                if pl.cube_near(start or [], c["u"], c["v"]) is not None
                and (not wanted or c["color"] in wanted)]
        self.publish_state(State.COMPLETE, placed=placed, left_on_table=left)


def main():
    rclpy.init()
    node = LearnedPickDemo()
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
