"""Xbox / Gamepad Joystick Teleoperation & Gripper Calibration Node for Braccio.

Provides smooth manual control over the Braccio arm and dedicated gripper
calibration features.

Controls (Standard Xbox 360 / One / Series Layout):
--------------------------------------------------
GRIPPER CALIBRATION:
  * RB (Right Bumper) : Close gripper incrementally (+1 deg / step)
  * LB (Left Bumper)  : Open gripper incrementally (-1 deg / step)
  * Button A          : Snap gripper to fully OPEN (10 deg)
  * Button B          : Snap gripper to default CLOSED (95 deg)
  * Button Y          : PRINT CALIBRATION REPORT to terminal with copy-paste values!
  * Button X          : Return arm to READY home pose

ARM MOVEMENT MODES (Toggle with BACK / VIEW button):
  [Mode 1: Cartesian IK (Default)]
    * Left Stick      : Move End-Effector in X (forward/back) & Y (left/right)
    * Right Stick V   : Move End-Effector in Z (elevation up/down)
    * Right Stick H   : Rotate Wrist
    * D-Pad           : Precision micro-jogging in X/Y/Z (1 mm steps)

  [Mode 2: Joint Jogging]
    * Left Stick      : Base (Horiz) & Shoulder (Vert)
    * Right Stick     : Wrist Vertical (Horiz) & Elbow (Vert)
    * D-Pad Horiz     : Wrist Rotation

SPEED:
  * START / MENU button: Toggle Speed (Normal vs Precision Slow Mode)
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy

from unoq_braccio_driver.braccio_kinematics import (
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    forward_kinematics,
    solve_ik,
)
from unoq_braccio_driver.braccio_model import (
    JOINT_LIMITS,
    JOINT_NAMES,
    POSES,
    clamp_degrees,
)


class JoystickTeleop(Node):
    def __init__(self) -> None:
        super().__init__("unoq_braccio_joystick_teleop")

        # Configurable Parameters
        self.declare_parameter("command_topic", "/braccio/joint_command")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("publish_rate", 20.0)  # Hz
        self.declare_parameter("ik_linear_speed", 0.05)  # m/s in Cartesian mode
        self.declare_parameter("joint_speed", 45.0)  # deg/s in Joint mode

        topic = self.get_parameter("command_topic").get_parameter_value().string_value
        joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value

        self.publisher = self.create_publisher(JointState, topic, 10)
        self.subscription = self.create_subscription(Joy, joy_topic, self.on_joy, 10)

        # Current State
        # Joint angles in degrees: [base, shoulder, elbow, wrist_v, wrist_r, gripper]
        self.joints = [float(v) for v in POSES["ready"]]
        self.cartesian_mode = True
        self.precision_mode = False

        # Current end-effector pose target (meters)
        self.ee_x = 0.22
        self.ee_y = 0.00
        self.ee_z = 0.08
        self.wrist_rot = 90.0

        # Input state caching
        self.last_joy_time = time.monotonic()
        self.latest_joy = None
        self.last_buttons = []
        self.last_report_print = 0.0

        # Periodic command publisher timer
        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info("==================================================")
        self.get_logger().info("🎮 BRACCIO JOYSTICK TELEOP & CALIBRATION NODE READY")
        self.get_logger().info("   LB/RB: Step Gripper | Y: Print Calibration")
        self.get_logger().info("   Left/Right Sticks: Move Arm | Back: Switch Mode")
        self.get_logger().info("==================================================")

    def on_joy(self, msg: Joy) -> None:
        self.latest_joy = msg

        # Edge-triggered button actions
        if not self.last_buttons or len(self.last_buttons) != len(msg.buttons):
            self.last_buttons = list(msg.buttons)
            return

        def pressed(btn_idx: int) -> bool:
            return bool(msg.buttons[btn_idx] and not self.last_buttons[btn_idx])

        # Button 6 (Back/View): Toggle Cartesian IK vs Joint mode
        if len(msg.buttons) > 6 and pressed(6):
            self.cartesian_mode = not self.cartesian_mode
            mode_str = "CARTESIAN IK (XYZ)" if self.cartesian_mode else "JOINT JOGGING (deg)"
            self.get_logger().info(f"🕹️ Switched Control Mode to: {mode_str}")

        # Button 7 (Start/Menu): Toggle precision speed mode
        if len(msg.buttons) > 7 and pressed(7):
            self.precision_mode = not self.precision_mode
            speed_str = "PRECISION (Slow)" if self.precision_mode else "NORMAL"
            self.get_logger().info(f"⚡ Speed Mode: {speed_str}")

        # Button 0 (A): Snap Gripper OPEN
        if len(msg.buttons) > 0 and pressed(0):
            self.joints[5] = float(GRIPPER_OPEN)
            self.get_logger().info(f"👐 Gripper Snap OPEN ({GRIPPER_OPEN}°)")

        # Button 1 (B): Snap Gripper CLOSED
        if len(msg.buttons) > 1 and pressed(1):
            self.joints[5] = float(GRIPPER_CLOSED)
            self.get_logger().info(f"✊ Gripper Snap CLOSED ({GRIPPER_CLOSED}°)")

        # Button 2 (X): Reset to READY pose
        if len(msg.buttons) > 2 and pressed(2):
            self.joints = [float(v) for v in POSES["ready"]]
            self.ee_x, self.ee_y, self.ee_z = 0.22, 0.00, 0.08
            self.wrist_rot = 90.0
            self.get_logger().info("🏠 Reset Arm to READY pose")

        # Button 3 (Y): Print Calibration Report
        if len(msg.buttons) > 3 and pressed(3):
            self.print_calibration_report()

        self.last_buttons = list(msg.buttons)

    def apply_deadzone(self, val: float) -> float:
        dz = float(self.get_parameter("deadzone").value)
        if abs(val) < dz:
            return 0.0
        # Re-scale smoothly outside deadzone
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - dz) / (1.0 - dz)

    def control_loop(self) -> None:
        now = time.monotonic()
        dt = min(0.1, max(0.001, now - self.last_joy_time))
        self.last_joy_time = now

        if self.latest_joy is None:
            return

        axes = self.latest_joy.axes
        buttons = self.latest_joy.buttons

        speed_scale = 0.25 if self.precision_mode else 1.0

        # --- Gripper Control (LB / RB & Triggers) ---
        # RB (button 5) = close (+deg), LB (button 4) = open (-deg)
        grip_step = 25.0 * dt * speed_scale  # degrees per sec
        if len(buttons) > 5 and buttons[5]:  # RB
            self.joints[5] += grip_step
        if len(buttons) > 4 and buttons[4]:  # LB
            self.joints[5] -= grip_step

        # D-pad vertical micro-steps on gripper if in precision mode
        if len(axes) > 7 and abs(axes[7]) > 0.5:
            self.joints[5] += (axes[7] * 10.0 * dt)

        # Clamp gripper within valid hardware range
        self.joints[5] = float(clamp_degrees("gripper", self.joints[5]))

        # --- Arm Positioning ---
        if self.cartesian_mode:
            # Axis 0: Left Stick Horiz (Left = +Y, Right = -Y)
            # Axis 1: Left Stick Vert  (Up = +X, Down = -X)
            # Axis 4: Right Stick Vert (Up = +Z, Down = -Z)
            # Axis 3: Right Stick Horiz (Wrist Rotation)
            lx = self.apply_deadzone(axes[0] if len(axes) > 0 else 0.0)
            ly = self.apply_deadzone(axes[1] if len(axes) > 1 else 0.0)
            rz = self.apply_deadzone(axes[4] if len(axes) > 4 else 0.0)
            rw = self.apply_deadzone(axes[3] if len(axes) > 3 else 0.0)

            v_lin = float(self.get_parameter("ik_linear_speed").value) * speed_scale
            dx = ly * v_lin * dt
            dy = -lx * v_lin * dt
            dz = rz * v_lin * dt
            dw = rw * 45.0 * dt * speed_scale

            new_x = self.ee_x + dx
            new_y = self.ee_y + dy
            new_z = max(0.005, self.ee_z + dz)
            new_wrist = clamp_degrees("wrist_rotation", self.wrist_rot + dw)

            # Solve IK for new coordinates
            sol = solve_ik(new_x, new_y, new_z, gripper=int(self.joints[5]), wrist_rotation=new_wrist)
            if sol is not None:
                self.ee_x = new_x
                self.ee_y = new_y
                self.ee_z = new_z
                self.wrist_rot = float(new_wrist)
                for i in range(5):
                    self.joints[i] = float(sol[i])
        else:
            # Direct Joint Jogging Mode
            # Left Stick: Base (H) & Shoulder (V)
            # Right Stick: Wrist Vertical (H) & Elbow (V)
            # D-Pad Horiz: Wrist Rotation
            v_joint = float(self.get_parameter("joint_speed").value) * speed_scale
            j_base = self.apply_deadzone(axes[0] if len(axes) > 0 else 0.0)
            j_shld = self.apply_deadzone(axes[1] if len(axes) > 1 else 0.0)
            j_elbw = self.apply_deadzone(axes[4] if len(axes) > 4 else 0.0)
            j_wr_v = self.apply_deadzone(axes[3] if len(axes) > 3 else 0.0)
            j_wr_r = axes[6] if len(axes) > 6 else 0.0

            self.joints[0] = clamp_degrees("base", self.joints[0] - j_base * v_joint * dt)
            self.joints[1] = clamp_degrees("shoulder", self.joints[1] - j_shld * v_joint * dt)
            self.joints[2] = clamp_degrees("elbow", self.joints[2] - j_elbw * v_joint * dt)
            self.joints[3] = clamp_degrees("wrist_vertical", self.joints[3] + j_wr_v * v_joint * dt)
            self.joints[4] = clamp_degrees("wrist_rotation", self.joints[4] + j_wr_r * v_joint * dt)

            # Update FK representation
            try:
                tip_xyz = forward_kinematics(self.joints[:5])
                self.ee_x, self.ee_y, self.ee_z = tip_xyz[0], tip_xyz[1], tip_xyz[2]
            except Exception:
                pass

        # Publish JointState command
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(clamp_degrees(name, self.joints[i])) for i, name in enumerate(JOINT_NAMES)]
        self.publisher.publish(msg)

        # Status HUD in terminal (throttled every 0.5s)
        if now - self.last_report_print > 0.5:
            self.last_report_print = now
            self.render_hud()

    def render_hud(self) -> None:
        mode_tag = "IK-XYZ" if self.cartesian_mode else "JOINT "
        speed_tag = "SLOW" if self.precision_mode else "NORM"
        grip_val = int(round(self.joints[5]))
        # Visual progress bar for gripper: 10 to 110 deg
        bar_len = 10
        filled = int(round(((grip_val - 10) / 100.0) * bar_len))
        filled = max(0, min(bar_len, filled))
        grip_bar = "[" + "=" * filled + ">" + " " * (bar_len - filled) + "]"

        status = (
            f"[{mode_tag}|{speed_tag}] "
            f"XYZ: ({self.ee_x:.3f}, {self.ee_y:.3f}, {self.ee_z:.3f})m | "
            f"GRIPPER: {grip_bar} {grip_val:3d}° | Press Y to Calibrate"
        )
        self.get_logger().info(status)

    def print_calibration_report(self) -> None:
        grip_deg = int(round(self.joints[5]))
        tip_xyz = forward_kinematics(self.joints[:5])
        angles_str = ", ".join(f"{name}={int(round(self.joints[i]))}" for i, name in enumerate(JOINT_NAMES))

        print("\n" + "=" * 65)
        print("🎯 BRACCIO GRIPPER & POSE CALIBRATION REPORT")
        print("=" * 65)
        print(f"Current Gripper Angle : {grip_deg}° (Range: 10° open -> 110° fully shut)")
        print(f"Fingertip XYZ (World) : X = {tip_xyz[0]:.4f} m, Y = {tip_xyz[1]:.4f} m, Z = {tip_xyz[2]:.4f} m")
        print(f"Full Joint Vector     : [{angles_str}]")
        print("-" * 65)
        print("📌 COPY & PASTE INTO CODE TO CALIBRATE:")
        print(f"  In 'ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_kinematics.py':")
        print(f"      GRIPPER_CLOSED = {grip_deg}")
        print(f"\n  In 'ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_workspace.py':")
        print(f"      CUBE_CENTRE_Z = {round(tip_xyz[2], 3)}")
        print("=" * 65 + "\n")


def main() -> None:
    rclpy.init()
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
