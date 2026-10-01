import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from unoq_braccio_driver.braccio_kinematics import (
    URDF_JOINT_NAMES,
    servo_positions_to_urdf,
)
from unoq_braccio_driver.braccio_model import JOINT_NAMES, POSES


class JointTrajectoryBridge(Node):
    """/braccio/joint_command (servo degrees) -> /arm_controller/joint_trajectory.

    A command that arrives before the trajectory controller is listening (the
    controllers start a few seconds after Gazebo) would otherwise be lost, so
    it is held until a subscriber exists, then sent twice to cover the gap
    between the controller subscribing and becoming active. A command sent
    while the controller is already up is sent once, so moves are not restarted.

    The move time scales with the size of the move (max_joint_speed), capped
    at move_time: big pose changes stay slow and smooth, while the small,
    frequent steps streamed by manual_control track the stick without lag.
    """

    def __init__(self) -> None:
        super().__init__("unoq_braccio_joint_trajectory_bridge")
        self.declare_parameter("move_time", 1.5)  # longest move, seconds
        self.declare_parameter("min_move_time", 0.1)
        self.declare_parameter("max_joint_speed", 1.2)  # rad/s
        self.declare_parameter("late_repeat_period", 1.0)  # gap between the two sends
        self.last_servo = {name: float(POSES["ready"][i]) for i, name in enumerate(JOINT_NAMES)}
        self.have_command = False  # until the first command, the arm pose is unknown
        self.pending = None
        self.sends_left = 0
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )
        self.subscription = self.create_subscription(
            JointState,
            "/braccio/joint_command",
            self.on_command,
            10,
        )
        self.timer = self.create_timer(0.25, self.flush)
        self.last_send = 0.0

    def on_command(self, msg: JointState) -> None:
        # Joints missing from the message keep their last commanded value.
        previous = servo_positions_to_urdf(self.last_servo)
        self.last_servo.update(dict(zip(msg.name, (float(v) for v in msg.position))))

        trajectory = JointTrajectory()
        # A zero stamp means "start now" for the controller, whatever clock it uses.
        trajectory.joint_names = URDF_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = servo_positions_to_urdf(self.last_servo)
        move_time = float(self.get_parameter("move_time").value)
        if self.have_command:
            largest = max(abs(a - b) for a, b in zip(point.positions, previous))
            move_time = min(
                move_time,
                max(
                    float(self.get_parameter("min_move_time").value),
                    largest / float(self.get_parameter("max_joint_speed").value),
                ),
            )
        self.have_command = True
        point.time_from_start = Duration(
            sec=int(move_time), nanosec=int((move_time % 1.0) * 1e9)
        )
        trajectory.points = [point]

        self.pending = trajectory
        # Controller already listening: one send. Not yet: hold, then send twice.
        self.sends_left = 1 if self.publisher.get_subscription_count() > 0 else 2
        self.last_send = 0.0
        self.flush()

    def flush(self) -> None:
        if self.pending is None or self.sends_left <= 0:
            return
        if self.publisher.get_subscription_count() == 0:
            return  # controller not up yet; keep the command and retry
        now = time.monotonic()  # wall clock: must not depend on /clock
        if now - self.last_send < float(self.get_parameter("late_repeat_period").value):
            return
        self.publisher.publish(self.pending)
        self.last_send = now
        self.sends_left -= 1


def main() -> None:
    rclpy.init()
    node = JointTrajectoryBridge()
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
