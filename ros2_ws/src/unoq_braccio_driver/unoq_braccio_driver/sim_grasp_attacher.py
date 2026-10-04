"""Simulation-only grasp assist: weld the grasped cube to the wrist.

Finger friction alone does not hold a cube reliably in Gazebo, so each cube
has a DetachableJoint in braccio.urdf.xacro. This node drives them from one
simple request topic, so the pick/place logic never deals with Gazebo:

    /braccio/grasp (std_msgs/String)
        "grasp:<colour>"  weld that cube to the wrist (fingers already closed)
        "release"         let go of whatever is held

On hardware nothing listens to /braccio/grasp, so the same pick/place node
works unchanged.

It reconciles rather than fires once: every tick, each cube that is not in
the wanted state (known from the plugin's state topic) gets another
attach/detach request. The plugin ignores a request for the state it is
already in, so repeats are harmless. This also covers Gazebo Harmonic's
DetachableJoint attaching every cube as soon as the robot spawns: they are
all wanted detached, so they are let go again straight away.

The current grasp is published on /braccio/grasp_state (JSON).
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from unoq_braccio_driver import braccio_workspace as ws


class SimGraspAttacher(Node):
    def __init__(self) -> None:
        super().__init__("sim_grasp_attacher")
        self.declare_parameter("reconcile_period", 0.2)

        self.colors = list(ws.CUBES)
        self.wanted = {c: False for c in self.colors}   # True = should be attached
        self.known = {c: None for c in self.colors}     # last plugin state, None = unknown
        self.attach_pub = {}
        self.detach_pub = {}
        for color in self.colors:
            self.attach_pub[color] = self.create_publisher(
                Empty, ws.grasp_topic(color, "attach"), 10
            )
            self.detach_pub[color] = self.create_publisher(
                Empty, ws.grasp_topic(color, "detach"), 10
            )
            self.create_subscription(
                String,
                ws.grasp_topic(color, "state"),
                lambda msg, c=color: self.on_state(c, msg),
                10,
            )
        self.state_pub = self.create_publisher(String, "/braccio/grasp_state", 10)
        self.create_subscription(String, "/braccio/grasp", self.on_request, 10)
        self.create_timer(float(self.get_parameter("reconcile_period").value), self.reconcile)

    def on_request(self, msg: String) -> None:
        request = msg.data.strip().lower()
        if request == "release":
            for color in self.colors:
                self.wanted[color] = False
            self.get_logger().info("release")
        elif request.startswith("grasp:"):
            color = request.split(":", 1)[1].strip()
            if color not in self.wanted:
                self.get_logger().warning(f"grasp: unknown cube colour '{color}'")
                return
            # One cube at a time.
            for other in self.colors:
                self.wanted[other] = other == color
            self.get_logger().info(f"grasp {ws.cube_model(color)}")
        else:
            self.get_logger().warning(f"unknown grasp request '{msg.data}'")
            return
        self.reconcile()

    def on_state(self, color: str, msg: String) -> None:
        if self.known[color] != msg.data:
            self.get_logger().info(f"{ws.cube_model(color)}: {msg.data}")
        self.known[color] = msg.data
        self.publish_state()

    def reconcile(self) -> None:
        for color in self.colors:
            if self.wanted[color] and self.known[color] != "attached":
                self.attach_pub[color].publish(Empty())
            elif not self.wanted[color] and self.known[color] != "detached":
                self.detach_pub[color].publish(Empty())

    def publish_state(self) -> None:
        held = [c for c in self.colors if self.known[c] == "attached"]
        self.state_pub.publish(String(data=json.dumps({"held": held, "states": self.known})))


def main() -> None:
    rclpy.init()
    node = SimGraspAttacher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
