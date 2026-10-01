"""Bridges JointState from joint_state_publisher_gui (radians) to /braccio/joint_command (servo degrees).

Enables using the ROS 2 joint_state_publisher_gui slider tool to interactively
control the Braccio arm in Gazebo simulation or on real hardware, and see the
exact calibrated servo angles in real time.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from unoq_braccio_driver.braccio_kinematics import (
    GRIPPER_RAD_MAX,
    GRIPPER_RAD_MIN,
)
from unoq_braccio_driver.braccio_model import JOINT_LIMITS, JOINT_NAMES, clamp_degrees


def urdf_rad_to_servo(name: str, rad: float) -> float:
    """Convert URDF joint angle (radians) to Braccio servo degrees (0-180)."""
    if name in ("shoulder", "elbow", "wrist_vertical"):
        return 180.0 - math.degrees(rad)
    if name in ("base", "wrist_rotation"):
        return 90.0 + math.degrees(rad)
    if name == "gripper":
        span = GRIPPER_RAD_MAX - GRIPPER_RAD_MIN
        frac = (rad - GRIPPER_RAD_MIN) / span if span > 1e-6 else 0.0
        return 10.0 + max(0.0, min(1.0, frac)) * 100.0
    return 90.0


class GuiToJointCommand(Node):
    def __init__(self) -> None:
        super().__init__("gui_to_joint_command")
        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/braccio/joint_command")

        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self.subscription = self.create_subscription(JointState, in_topic, self.on_joint_state, 10)
        self.publisher = self.create_publisher(JointState, out_topic, 10)
        self.last_print = 0.0

        self.get_logger().info(f"Bridging '{in_topic}' (rad) -> '{out_topic}' (servo deg)")

    def on_joint_state(self, msg: JointState) -> None:
        name_map = dict(zip(msg.name, msg.position))
        degrees_map = {}
        for jname in JOINT_NAMES:
            if jname in name_map:
                deg = urdf_rad_to_servo(jname, name_map[jname])
                degrees_map[jname] = float(clamp_degrees(jname, deg))

        if not degrees_map:
            return

        out_msg = JointState()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.name = list(degrees_map.keys())
        out_msg.position = list(degrees_map.values())
        self.publisher.publish(out_msg)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_print > 0.5:
            self.last_print = now
            grip_deg = int(round(degrees_map.get("gripper", 10)))
            self.get_logger().info(
                f"[GUI SLIDER] Grip: {grip_deg:3d}° | Base: {int(degrees_map.get('base', 90)):3d}° | "
                f"Shoulder: {int(degrees_map.get('shoulder', 90)):3d}° | Elbow: {int(degrees_map.get('elbow', 90)):3d}° | "
                f"WristV: {int(degrees_map.get('wrist_vertical', 90)):3d}° | WristR: {int(degrees_map.get('wrist_rotation', 90)):3d}°"
            )


def main() -> None:
    rclpy.init()
    node = GuiToJointCommand()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
