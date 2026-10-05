"""Camera-driven pick and place task manager.

The overhead camera reports where cubes and bins are. The gripper camera only
confirms what the gripper is looking at. Each cube is taken through an explicit
state machine and dropped in the bin that matches its colour.

    ros2 run unoq_braccio_driver pick_place_demo
    ros2 run unoq_braccio_driver pick_place_demo --ros-args -p colors:="[blue]"

The defaults are the simulation. On the real arm (real_pick_place.launch.py)
``workspace_config`` loads the real cube size, bins and gripper values, and
``use_gripper_camera:=false`` skips the two gripper-camera checks.

State (String) is published on /task/state and details (JSON) on /task/current.
Commands go out on /braccio/joint_command, so the same node drives Gazebo or
the hardware bridge. Grasp/release is also announced on /braccio/grasp; in
simulation sim_grasp_attacher welds the cube to the wrist on that, on
hardware nothing listens and the fingers alone hold the cube.
"""

import json
import os
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from unoq_braccio_driver import braccio_workspace as ws
from unoq_braccio_driver.braccio_kinematics import grasp_wrist_rotation, solve_ik
from unoq_braccio_driver.braccio_model import JOINT_NAMES, POSES


class State(str, Enum):
    IDLE = "IDLE"
    GO_HOME = "GO_HOME"
    DETECTING = "DETECTING"
    TARGET_CONFIRMED = "TARGET_CONFIRMED"
    MOVE_ABOVE_CUBE = "MOVE_ABOVE_CUBE"
    VERIFY_CUBE = "VERIFY_CUBE"          # gripper camera sees the right colour
    DESCEND = "DESCEND"
    GRASP = "GRASP"
    LIFT = "LIFT"
    VERIFY_GRASP = "VERIFY_GRASP"        # gripper camera sees the cube held
    MOVE_TO_BIN = "MOVE_TO_BIN"
    LOWER = "LOWER"
    RELEASE = "RELEASE"
    RETREAT = "RETREAT"
    VERIFY_PLACEMENT = "VERIFY_PLACEMENT"  # overhead camera sees it in the bin
    COMPLETE = "COMPLETE"
    # error states
    DETECTION_FAILED = "DETECTION_FAILED"
    TARGET_UNREACHABLE = "TARGET_UNREACHABLE"
    VERIFY_FAILED = "VERIFY_FAILED"


class PickPlaceDemo(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_demo")
        self.declare_parameter("colors", ["red", "blue", "yellow"])
        self.declare_parameter("step_wait", 2.0)      # >= joint_trajectory_bridge move_time
        self.declare_parameter("gripper_wait", 2.0)   # let the fingers close and settle before lifting
        self.declare_parameter("detect_timeout", 6.0)
        # False: a failed gripper-camera check only warns. True: it aborts the cube.
        self.declare_parameter("strict_gripper_verify", False)
        self.declare_parameter("min_grasp_area_frac", 0.02)
        self.declare_parameter("grasp_settle", 0.3)   # let the sim weld the cube before lifting
        self.declare_parameter("use_gripper_camera", True)
        self.declare_parameter("workspace_config", "")  # "" = simulated layout

        config = str(self.get_parameter("workspace_config").value)
        if config:
            ws.load_config(os.path.expanduser(config))
            self.get_logger().info(f"Workspace from {config}")

        self.command = self.create_publisher(JointState, "/braccio/joint_command", 10)
        self.request = self.create_publisher(String, "/vision/detect_request", 10)
        self.gripper_request = self.create_publisher(String, "/vision/gripper/detect_request", 10)
        self.grasp_pub = self.create_publisher(String, "/braccio/grasp", 10)
        self.state_pub = self.create_publisher(String, "/task/state", 10)
        self.current_pub = self.create_publisher(String, "/task/current", 10)
        self.create_subscription(String, "/vision/cube_target", self.on_overhead, 10)
        self.create_subscription(String, "/vision/gripper/detection", self.on_gripper, 10)

        self.overhead = None
        self.overhead_event = threading.Event()
        self.gripper = None
        self.gripper_event = threading.Event()
        self.state = State.IDLE
        self.current = {}
        self.slot_used = {}
        self.publish_state(State.IDLE)

    # -- I/O ---------------------------------------------------------

    def on_overhead(self, msg: String) -> None:
        try:
            self.overhead = json.loads(msg.data)
        except json.JSONDecodeError:
            self.overhead = {"cubes": [], "bins": {}}
        self.overhead_event.set()

    def on_gripper(self, msg: String) -> None:
        try:
            self.gripper = json.loads(msg.data)
        except json.JSONDecodeError:
            self.gripper = {"colors": {}}
        self.gripper_event.set()

    def publish_state(self, state: State, **details) -> None:
        self.state = state
        self.current.update(details)
        self.current["state"] = state.value
        self.state_pub.publish(String(data=state.value))
        self.current_pub.publish(String(data=json.dumps(self.current)))
        self.get_logger().info(f"[{state.value}] {details or ''}")

    def move(self, pose, wait=None) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(v) for v in pose]
        self.command.publish(msg)
        time.sleep(float(self.get_parameter("step_wait").value) if wait is None else wait)

    def grasp(self, color: str) -> None:
        """Announce that ``color`` is between the closed fingers (sim welds it)."""
        self.grasp_pub.publish(String(data=f"grasp:{color}"))
        time.sleep(float(self.get_parameter("grasp_settle").value))

    def release(self) -> None:
        """Announce that the held cube is being let go (sim unwelds it)."""
        self.grasp_pub.publish(String(data="release"))
        time.sleep(float(self.get_parameter("grasp_settle").value))

    def ik(self, x, y, z, gripper, wrist_rotation=90):
        pose = solve_ik(x, y, z, gripper, wrist_rotation)
        if pose is None:
            raise ValueError(f"unreachable target ({x:.3f}, {y:.3f}, {z:.3f})")
        return pose

    def overhead_scan(self, color: str = ""):
        """Ask the overhead camera for cubes and bins; None on timeout."""
        self.overhead_event.clear()
        self.request.publish(String(data=color))
        if not self.overhead_event.wait(float(self.get_parameter("detect_timeout").value)):
            return None
        return self.overhead

    def gripper_look(self, color: str):
        """Ask the gripper camera what it sees of ``color``; None on timeout."""
        self.gripper_event.clear()
        self.gripper_request.publish(String(data=color))
        if not self.gripper_event.wait(float(self.get_parameter("detect_timeout").value)):
            return None
        return (self.gripper or {}).get("colors", {}).get(color)

    def gripper_check(self, state: State, color: str, min_frac: float) -> bool:
        """Gripper-camera confirmation. Only aborts when strict_gripper_verify."""
        if not bool(self.get_parameter("use_gripper_camera").value):
            return True  # no gripper camera: nothing to check
        seen = self.gripper_look(color)
        ok = seen is not None and seen["area_frac"] >= min_frac
        self.publish_state(state, gripper_seen=seen, gripper_ok=ok)
        if ok:
            return True
        self.get_logger().warning(f"Gripper camera did not confirm {color} ({state.value})")
        return not bool(self.get_parameter("strict_gripper_verify").value)

    # -- one cube ------------------------------------------------------

    def bin_target(self, cube_color: str, scan: dict):
        """Bin centre: overhead camera position when seen, workspace layout otherwise."""
        bin_ = ws.BIN_BY_CUBE_COLOR[cube_color]
        seen = (scan or {}).get("bins", {}).get(bin_.name)
        cx, cy = (seen["x"], seen["y"]) if seen else bin_.centre
        slot = self.slot_used.get(bin_.name, 0)
        self.slot_used[bin_.name] = slot + 1
        dx, dy = ws.BIN_SLOT_OFFSETS[slot % len(ws.BIN_SLOT_OFFSETS)]
        return bin_, cx + dx, cy + dy

    def handle_cube(self, cube: dict, scan: dict) -> bool:
        color, x, y = cube["color"], cube["x"], cube["y"]
        grasp_frac = float(self.get_parameter("min_grasp_area_frac").value)
        grip_wait = float(self.get_parameter("gripper_wait").value)
        self.publish_state(
            State.TARGET_CONFIRMED, cube=color, x=round(x, 4), y=round(y, 4),
            confidence=round(cube["confidence"], 2),
        )

        bin_, bx, by = self.bin_target(color, scan)
        try:
            # Roll the wrist so the fingers close square to the cube's faces
            # (cubes sit axis-aligned) rather than across its diagonal, and
            # keep that roll for the whole pick so the cube is not twisted.
            roll = grasp_wrist_rotation(self.ik(x, y, ws.CUBE_CENTRE_Z, ws.GRIPPER_OPEN))
            above = self.ik(x, y, ws.HOVER_Z, ws.GRIPPER_OPEN, roll)
            down = self.ik(x, y, ws.CUBE_CENTRE_Z, ws.GRIPPER_OPEN, roll)
            grip = self.ik(x, y, ws.CUBE_CENTRE_Z, ws.GRIPPER_CLOSED, roll)
            lift = self.ik(x, y, ws.HOVER_Z, ws.GRIPPER_CLOSED, roll)
            over_bin = self.ik(bx, by, ws.HOVER_Z, ws.GRIPPER_CLOSED, roll)
            into_bin = self.ik(bx, by, ws.release_z(bin_), ws.GRIPPER_CLOSED, roll)
            release = self.ik(bx, by, ws.release_z(bin_), ws.GRIPPER_OPEN, roll)
            retreat = self.ik(bx, by, ws.HOVER_Z, ws.GRIPPER_OPEN, roll)
        except ValueError as exc:
            self.publish_state(State.TARGET_UNREACHABLE, error=str(exc))
            return False

        self.publish_state(State.MOVE_ABOVE_CUBE, bin=bin_.name)
        self.move(above)

        if not self.gripper_check(State.VERIFY_CUBE, color, min_frac=0.0005):
            self.publish_state(State.VERIFY_FAILED, error="gripper camera: cube not seen")
            return False

        self.publish_state(State.DESCEND)
        self.move(down)
        self.publish_state(State.GRASP)
        self.move(grip, wait=grip_wait)
        self.grasp(color)
        self.publish_state(State.LIFT)
        self.move(lift)

        if not self.gripper_check(State.VERIFY_GRASP, color, min_frac=grasp_frac):
            self.publish_state(State.VERIFY_FAILED, error="gripper camera: cube not held")
            self.release()
            self.move(self.ik(x, y, ws.HOVER_Z, ws.GRIPPER_OPEN))
            return False

        self.publish_state(State.MOVE_TO_BIN, bin=bin_.name, bin_x=round(bx, 4), bin_y=round(by, 4))
        self.move(over_bin)
        self.publish_state(State.LOWER)
        self.move(into_bin)
        self.publish_state(State.RELEASE)
        self.release()
        self.move(release, wait=grip_wait)
        self.publish_state(State.RETREAT)
        self.move(retreat)
        return True

    # -- whole task ------------------------------------------------------

    def run(self) -> None:
        time.sleep(1.0)  # let publishers and detectors discover each other
        wanted = [str(c).lower() for c in self.get_parameter("colors").value]
        home = list(POSES["ready"])
        home[5] = ws.GRIPPER_OPEN

        self.publish_state(State.GO_HOME)
        self.release()  # nothing should be held from an earlier, interrupted run
        self.move(home)

        self.publish_state(State.DETECTING)
        scan = self.overhead_scan()
        if scan is None:
            self.publish_state(State.DETECTION_FAILED, error="no overhead result")
            return
        todo = [
            c for c in scan["cubes"]
            if c["sector"] == "pick" and c["color"] in wanted and c["color"] in ws.BIN_BY_CUBE_COLOR
        ]
        if not todo:
            self.publish_state(State.DETECTION_FAILED, error="no cubes in the pick sector")
            return

        placed = []
        for cube in sorted(todo, key=lambda c: c["y"]):
            try:
                if self.handle_cube(cube, scan):
                    placed.append(cube["color"])
            except Exception as exc:  # keep going with the next cube
                self.get_logger().error(f"{cube['color']}: {exc}")
                self.release()
            self.publish_state(State.GO_HOME)
            self.move(home)

        self.publish_state(State.VERIFY_PLACEMENT, expected=placed)
        final = self.overhead_scan()
        bad = []
        if final is not None:
            for color in placed:
                target = ws.BIN_BY_CUBE_COLOR[color].name
                if not any(
                    c["color"] == color and c["sector"] == target for c in final["cubes"]
                ):
                    bad.append(color)
        if bad:
            self.publish_state(State.VERIFY_FAILED, error=f"not in bin: {bad}")
        else:
            self.publish_state(State.COMPLETE, placed=placed)


def main() -> None:
    rclpy.init()
    node = PickPlaceDemo()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
