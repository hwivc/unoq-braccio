"""Fun moves for the arm: wave, dance, nod, shake, bow.

    ros2 run unoq_braccio_driver arm_tricks --ros-args -p trick:=wave

Works on the real arm (hardware.launch.py / real.launch.py) and in Gazebo
(sim.launch.py): it only publishes /braccio/joint_command. Every trick ends
standing up straight.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from unoq_braccio_driver.braccio_model import JOINT_NAMES, START_POSE

UP = list(START_POSE)  # base, shoulder, elbow, wrist_vertical, wrist_rotation, gripper


def pose(**changes):
    """START_POSE with some joints changed, e.g. pose(base=60, gripper=10)."""
    values = dict(zip(JOINT_NAMES, UP))
    values.update(changes)
    return [values[name] for name in JOINT_NAMES]


# (pose, seconds to hold before the next one)
TRICKS = {
    "wave": [(pose(elbow=60, wrist_vertical=90, gripper=10), 1.2)]
    + [(pose(elbow=60, wrist_vertical=w, gripper=10), 0.5) for w in (50, 130) * 3],
    "dance": [(pose(base=b, shoulder=s, wrist_rotation=r, wrist_vertical=w), 0.6)
              for b, s, r, w in ((60, 100, 40, 70), (120, 80, 140, 110)) * 4],
    "nod": [(pose(shoulder=100, wrist_vertical=w), 0.45) for w in (60, 120) * 3],
    "shake": [(pose(shoulder=100, wrist_vertical=70, wrist_rotation=r), 0.45) for r in (50, 130) * 3],
    "bow": [(pose(shoulder=125, elbow=110, wrist_vertical=120), 1.8),
            (pose(gripper=10), 1.0), (pose(gripper=60), 0.6), (pose(gripper=10), 0.6)],
}


class ArmTricks(Node):
    def __init__(self):
        super().__init__("arm_tricks")
        self.declare_parameter("trick", "wave")
        self.declare_parameter("speed", 1.0)   # >1 faster, <1 slower
        self.publisher = self.create_publisher(JointState, "/braccio/joint_command", 10)

    def send(self, values):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in values]
        self.publisher.publish(msg)

    def run(self):
        name = str(self.get_parameter("trick").value).lower()
        if name not in TRICKS:
            self.get_logger().error(f"Unknown trick '{name}'. Try: {', '.join(TRICKS)}")
            return
        speed = max(0.2, float(self.get_parameter("speed").value))
        time.sleep(1.0)  # let the publisher connect
        self.get_logger().info(f"Doing: {name}")
        self.send(UP)
        time.sleep(1.5 / speed)
        for values, hold in TRICKS[name]:
            self.send(values)
            time.sleep(hold / speed)
        self.send(UP)
        time.sleep(1.5 / speed)


def main():
    rclpy.init()
    node = ArmTricks()
    try:
        node.run()
    except KeyboardInterrupt:
        node.send(UP)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
