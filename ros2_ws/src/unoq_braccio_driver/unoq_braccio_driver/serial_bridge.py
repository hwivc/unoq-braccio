"""USB serial bridge to an Arduino UNO running firmware/braccio_uno_firmware.

    /braccio/joint_command (JointState, servo degrees)  ->  "M ..." lines
    "STAT ..." replies  ->  /braccio/firmware_status (raw line)
                            /joint_states (URDF radians, for RViz / manual_control)

Commands are latest-wins: the firmware retargets smoothly mid-move, so when
commands arrive faster than min_command_period (manual_control streams them)
only the newest one is sent. Joints missing from a command keep their last
commanded angle (as in the simulator's joint_trajectory_bridge), starting from
where the firmware reports the arm is. The arm's real position is polled with "S"
every status_period.

Opening the port resets a classic UNO, and the firmware then soft-starts the
servos for about 6 s. Nothing is sent until the firmware has answered (its
READY banner, or a STAT reply to a probe), and the port is reopened if the
cable is pulled. After a reset the arm is back at its start pose; the last
command is deliberately not replayed.

The firmware powers up standing straight up (braccio_model.START_POSE). Until
the board reports its real position, /joint_states carries that start pose,
so RViz and manual_control begin from the same upright arm instead of an
empty or folded model.
"""

import threading
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from unoq_braccio_driver.braccio_kinematics import URDF_JOINT_NAMES, servo_positions_to_urdf
from unoq_braccio_driver.braccio_model import JOINT_NAMES, START_POSE, command_line_from_positions
from unoq_braccio_driver.braccio_protocol import (
    READY_PREFIX,
    parse_status,
    speed_command,
)


class SerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("braccio_serial_bridge")
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("command_topic", "/braccio/joint_command")
        self.declare_parameter("speed_deg_s", 60.0)       # fastest joint, degrees per second
        self.declare_parameter("min_command_period", 0.05)
        self.declare_parameter("status_period", 0.2)
        self.declare_parameter("boot_wait", 2.0)          # UNO bootloader after the port opens
        self.declare_parameter("reconnect_period", 2.0)
        self.declare_parameter("publish_joint_states", True)

        self.port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)

        self.lock = threading.Lock()   # guards self.serial and writes to it
        self.serial = None
        self.ready = False
        self.opened_at = 0.0
        self.last_open_attempt = -1e9
        self.last_probe = 0.0
        self.last_status_poll = 0.0
        self.last_command_sent = 0.0
        self.commanded = {}            # servo degrees by joint, from /braccio/joint_command
        self.baseline = dict(zip(JOINT_NAMES, START_POSE))  # where the firmware powers up
        self.have_status = False       # True once the board has reported its position
        self.dirty = False             # commanded has changed since the last send
        self.running = True

        self.status_pub = self.create_publisher(String, "/braccio/firmware_status", 10)
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(
            JointState, str(self.get_parameter("command_topic").value), self.on_command, 10
        )
        self.create_timer(0.02, self.tick)
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    # -- ROS side -----------------------------------------------------------

    def on_command(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            self.get_logger().warning("Ignoring empty JointState command")
            return
        self.commanded.update(zip(msg.name, (float(v) for v in msg.position)))
        self.dirty = True
        if not self.ready:
            self.get_logger().warning(
                "Arm not ready yet; the command will be sent once it is",
                throttle_duration_sec=2.0,
            )

    def tick(self) -> None:
        now = time.monotonic()
        if (not self.have_status
                and now - self.last_status_poll >= float(self.get_parameter("status_period").value)):
            # Nothing heard from the board yet: it is (re)starting into its
            # start pose, so report that pose until it says otherwise.
            self.last_status_poll = now
            if bool(self.get_parameter("publish_joint_states").value):
                self.publish_joint_states(START_POSE)

        if self.serial is None:
            if now - self.last_open_attempt >= float(self.get_parameter("reconnect_period").value):
                self.last_open_attempt = now
                self.open_port()
            return

        if not self.ready:
            # The READY banner may have been missed (or this board does not
            # reset on open): probe with S after the bootloader has had its time.
            if (now - self.opened_at >= float(self.get_parameter("boot_wait").value)
                    and now - self.last_probe >= 1.0):
                self.last_probe = now
                self.write("S")
            return

        if (self.dirty
                and now - self.last_command_sent
                >= float(self.get_parameter("min_command_period").value)):
            self.dirty = False
            self.last_command_sent = now
            full = {**self.baseline, **self.commanded}
            self.write(command_line_from_positions(JOINT_NAMES, [full[n] for n in JOINT_NAMES]))

        if now - self.last_status_poll >= float(self.get_parameter("status_period").value):
            self.last_status_poll = now
            self.write("S")

    # -- serial side --------------------------------------------------------

    def open_port(self) -> None:
        try:
            port = serial.Serial(self.port, self.baud_rate, timeout=0.2)
        except serial.SerialException as error:
            self.get_logger().warning(
                f"Cannot open {self.port} ({error}); retrying", throttle_duration_sec=10.0
            )
            return
        with self.lock:
            self.serial = port
            self.ready = False
            self.opened_at = time.monotonic()
        self.get_logger().info(
            f"Opened {self.port} at {self.baud_rate} baud; waiting for the arm to power up"
        )

    def close_port(self, reason: str) -> None:
        with self.lock:
            if self.serial is None:
                return
            try:
                self.serial.close()
            except serial.SerialException:
                pass
            self.serial = None
            self.ready = False
            self.forget_commands()
        self.get_logger().error(f"Lost {self.port}: {reason}; reconnecting")

    def write(self, line: str) -> None:
        with self.lock:
            port = self.serial
            if port is None:
                return
            try:
                port.write((line + "\n").encode("ascii"))
            except serial.SerialException as error:
                write_error = error
            else:
                return
        self.close_port(str(write_error))

    def read_loop(self) -> None:
        while self.running:
            port = self.serial
            if port is None:
                time.sleep(0.1)
                continue
            try:
                raw = port.readline()
            except (serial.SerialException, OSError, TypeError) as error:
                # TypeError: pyserial raises it when the port is closed mid-read.
                self.close_port(str(error))
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if line:
                self.on_line(line)

    def on_line(self, line: str) -> None:
        if line.startswith(READY_PREFIX):
            if self.ready:
                self.get_logger().warning("Board reset: the arm is back at its start pose")
                self.forget_commands()
            self.mark_ready(line)
            return
        status = parse_status(line)
        if status is not None:
            if not self.ready:
                self.baseline = dict(zip(JOINT_NAMES, status["pos"]))
                self.mark_ready(line)
            self.have_status = True
            self.status_pub.publish(String(data=line))
            if bool(self.get_parameter("publish_joint_states").value):
                self.publish_joint_states(status["pos"])
        elif line.startswith("ERR"):
            self.get_logger().warning(f"Firmware: {line}")
        # OK and DONE need no action.

    def forget_commands(self) -> None:
        """After a reset the arm is at its start pose: drop what was commanded."""
        self.commanded.clear()
        self.dirty = False
        self.have_status = False
        self.baseline = dict(zip(JOINT_NAMES, START_POSE))

    def mark_ready(self, line: str) -> None:
        self.ready = True
        self.write(speed_command(float(self.get_parameter("speed_deg_s").value)))
        self.get_logger().info(f"Arm ready ({line})")

    def publish_joint_states(self, servo_degrees) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(URDF_JOINT_NAMES)
        msg.position = servo_positions_to_urdf(dict(zip(JOINT_NAMES, servo_degrees)))
        self.joint_state_pub.publish(msg)

    def destroy_node(self) -> bool:
        self.running = False
        with self.lock:
            if self.serial is not None:
                self.serial.close()
                self.serial = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SerialBridge()
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
