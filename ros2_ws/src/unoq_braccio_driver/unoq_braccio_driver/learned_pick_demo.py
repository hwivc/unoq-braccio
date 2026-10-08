"""Pick and place with a model taught by teach_pick. Start real.launch.py first.

    ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=desk
    ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=desk model:=model_0010

Looks at the table with the arm out of the way, picks the cube the model
knows best, checks with the camera that it left the table, drops it on the
taught drop pose for its colour, and repeats until no cube it can handle is
left. Cubes far from every example, or with no drop pose for their colour,
are left alone. Stop with Ctrl+C: the arm freezes where it is.

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

    def go_home(self, gripper=None):
        pose = list(self.home)
        if gripper is not None:
            pose[5] = gripper
        self.move(pose)

    def run(self):
        if self.model is None:
            self.get_logger().error(
                f"No model in {self.session.dir}/models. Teach one first: "
                f"ros2 run unoq_braccio_driver teach_pick --ros-args -p session:={self.session.name}")
            return
        self.get_logger().info(
            f"Session '{self.session.name}', {self.model.info.get('name')} "
            f"({self.model.info.get('chosen')}, {self.model.info.get('examples')} examples); "
            f"drop poses: {', '.join(self.drops) or 'none'}")
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
                if (guess["nearest_px"] <= unsure and cube["color"] in self.drops
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
            self.publish_state(State.MOVE_TO_BIN, bin=cube["color"])
            self.move(self.drops[cube["color"]][:5] + [self.gripper_closed])
            self.publish_state(State.RELEASE)
            self.move(self.drops[cube["color"]][:5] + [self.gripper_open], settle=0.5)
            placed.append(cube["color"])
        self.publish_state(State.GO_HOME)
        self.go_home()
        left = [c["color"] for c in self.read_cubes()
                if pl.cube_near(start or [], c["u"], c["v"]) is not None]
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
