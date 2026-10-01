"""Manual Control Node for Arduino UNO Q Braccio.

Supports two input types via the 'input_type' parameter:
  1. 'joystick' : USB Gamepad / Joystick (supports Ucom / DragonRise and Xbox controllers)
  2. 'keyboard' : Direct interactive terminal keyboard teleoperation

Features:
  - Precise Gripper Calibration (RB/LB or '['/']' to step 1 deg at a time).
  - One-click / one-button Calibration Report (press 'Y' on controller or 'y' on keyboard).
  - Dual modes: Cartesian IK (jog in X, Y, Z meters) and Joint Jogging (jog each servo).
  - Speed toggles: Normal vs Precision (fine-tuning).
  - Strictly respects Braccio joint limits and clamps.
"""

import math
import os
import sys
import threading
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

# Cross-platform single key reader for keyboard mode
if os.name == "nt":
    import msvcrt

    def get_key_nonblocking() -> str | None:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            try:
                return ch.decode("utf-8")
            except UnicodeDecodeError:
                return None
        return None
else:
    import select
    import termios
    import tty

    def get_key_nonblocking() -> str | None:
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None


class ManualControl(Node):
    def __init__(self) -> None:
        super().__init__("manual_control")

        # Parameters
        self.declare_parameter("input_type", "joystick")  # 'joystick' or 'keyboard'
        self.declare_parameter("controller_type", "ucom")  # 'ucom' or 'xbox'
        self.declare_parameter("command_topic", "/braccio/joint_command")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("publish_rate", 20.0)  # Hz
        self.declare_parameter("ik_linear_speed", 0.05)  # m/s
        self.declare_parameter("joint_speed", 45.0)  # deg/s
        self.declare_parameter("deadzone", 0.12)

        self.input_type = self.get_parameter("input_type").get_parameter_value().string_value.lower()
        self.controller_type = self.get_parameter("controller_type").get_parameter_value().string_value.lower()
        cmd_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value

        self.publisher = self.create_publisher(JointState, cmd_topic, 10)

        # State Variables
        self.joints = [float(v) for v in POSES["ready"]]
        self.cartesian_mode = True
        self.precision_mode = False

        self.ee_x = 0.22
        self.ee_y = 0.00
        self.ee_z = 0.08
        self.wrist_rot = 90.0

        self.last_loop_time = time.monotonic()
        self.latest_joy = None
        self.last_buttons = []
        self.last_hud_print = 0.0
        self.running = True

        # Keyboard setup
        self.orig_term_settings = None
        if self.input_type == "keyboard":
            if os.name != "nt":
                try:
                    self.orig_term_settings = termios.tcgetattr(sys.stdin)
                    tty.setcbreak(sys.stdin.fileno())
                except Exception as e:
                    self.get_logger().warn(f"Could not set raw terminal mode: {e}")
            self.print_keyboard_help()
        else:
            self.subscription = self.create_subscription(Joy, joy_topic, self.on_joy, 10)
            self.print_joystick_help()

        # Timer loop
        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(period, self.control_loop)

    def print_keyboard_help(self) -> None:
        help_text = """
======================================================================
⌨️  BRACCIO MANUAL CONTROL: KEYBOARD MODE
======================================================================
  [ARM POSITIONING - CARTESIAN (IK)]
    W / S : Forward (+X) / Backward (-X)
    A / D : Left (+Y) / Right (-Y)
    R / F : Up (+Z) / Down (-Z)
    Q / E : Rotate Wrist CCW / CW

  [GRIPPER CALIBRATION]
    [ / ] : Micro-step Gripper Open (-1 deg) / Close (+1 deg)
    O     : Snap Gripper fully OPEN (10 deg)
    C     : Snap Gripper default CLOSED (95 deg)
    Y     : 📋 PRINT FULL CALIBRATION REPORT

  [UTILITY]
    M     : Toggle Control Mode (Cartesian IK <-> Joint Jogging)
    P     : Toggle Speed (Normal <-> Precision Slow)
    H     : Home / Reset arm to READY pose
    ?     : Show this help menu
    Ctrl+C: Exit
======================================================================
"""
        print(help_text)

    def print_joystick_help(self) -> None:
        is_ucom = (self.controller_type == "ucom")
        help_text = f"""
======================================================================
🎮 BRACCIO MANUAL CONTROL: JOYSTICK MODE ({'UCOM / USB GAMEPAD' if is_ucom else 'XBOX'})
======================================================================
  [GRIPPER CALIBRATION]
    {'R1 / Button 6' if is_ucom else 'RB'} : Step Gripper CLOSE (+1 deg)
    {'L1 / Button 5' if is_ucom else 'LB'} : Step Gripper OPEN (-1 deg)
    {'Button 3 (X)' if is_ucom else 'Button A'} : Snap OPEN (10 deg)
    {'Button 2 (O)' if is_ucom else 'Button B'} : Snap CLOSED (95 deg)
    {'Button 1 (▲)' if is_ucom else 'Button Y'} : 📋 PRINT FULL CALIBRATION REPORT

  [ARM POSITIONING]
    Left Stick   : Move X (Forward/Back) & Y (Left/Right)
    Right Stick  : Vertical moves Z (Elevation); Horizontal rotates Wrist
    {'Select / Button 9' if is_ucom else 'Back/View'} : Toggle Mode (Cartesian IK <-> Joint Jog)
    {'Start / Button 10' if is_ucom else 'Start'}   : Toggle Speed (Normal <-> Precision)
    {'Button 4 (■)' if is_ucom else 'Button X'} : Home / Reset to READY pose
======================================================================
"""
        print(help_text)

    def on_joy(self, msg: Joy) -> None:
        self.latest_joy = msg
        if not self.last_buttons or len(self.last_buttons) != len(msg.buttons):
            self.last_buttons = list(msg.buttons)
            return

        def pressed(btn: int) -> bool:
            return bool(btn < len(msg.buttons) and msg.buttons[btn] and not self.last_buttons[btn])

        is_ucom = (self.controller_type == "ucom")

        # Mode switch: Ucom button 8 (Select) or Xbox button 6 (Back)
        mode_btn = 8 if is_ucom else 6
        if pressed(mode_btn):
            self.cartesian_mode = not self.cartesian_mode
            name = "CARTESIAN IK (XYZ)" if self.cartesian_mode else "JOINT JOGGING"
            self.get_logger().info(f"🕹️ Mode Switched: {name}")

        # Speed switch: Ucom button 9 (Start) or Xbox button 7 (Start)
        speed_btn = 9 if is_ucom else 7
        if pressed(speed_btn):
            self.precision_mode = not self.precision_mode
            name = "PRECISION (Slow)" if self.precision_mode else "NORMAL"
            self.get_logger().info(f"⚡ Speed: {name}")

        # Snap Open: Ucom button 2 (X) or Xbox button 0 (A)
        snap_open_btn = 2 if is_ucom else 0
        if pressed(snap_open_btn):
            self.joints[5] = float(GRIPPER_OPEN)
            self.get_logger().info(f"👐 Gripper OPEN ({GRIPPER_OPEN}°)")

        # Snap Closed: Ucom button 1 (Circle) or Xbox button 1 (B)
        snap_closed_btn = 1 if is_ucom else 1
        if pressed(snap_closed_btn):
            self.joints[5] = float(GRIPPER_CLOSED)
            self.get_logger().info(f"✊ Gripper CLOSED ({GRIPPER_CLOSED}°)")

        # Calibration Report: Ucom button 0 (Triangle) or Xbox button 3 (Y)
        calib_btn = 0 if is_ucom else 3
        if pressed(calib_btn):
            self.print_calibration_report()

        # Reset Ready: Ucom button 3 (Square) or Xbox button 2 (X)
        reset_btn = 3 if is_ucom else 2
        if pressed(reset_btn):
            self.joints = [float(v) for v in POSES["ready"]]
            self.ee_x, self.ee_y, self.ee_z = 0.22, 0.00, 0.08
            self.wrist_rot = 90.0
            self.get_logger().info("🏠 Reset Arm to READY pose")

        self.last_buttons = list(msg.buttons)

    def apply_deadzone(self, val: float) -> float:
        dz = float(self.get_parameter("deadzone").value)
        if abs(val) < dz:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - dz) / (1.0 - dz)

    def control_loop(self) -> None:
        now = time.monotonic()
        dt = min(0.1, max(0.001, now - self.last_loop_time))
        self.last_loop_time = now

        if self.input_type == "keyboard":
            self.handle_keyboard_input(dt)
        else:
            self.handle_joystick_input(dt)

        # Publish JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(clamp_degrees(name, self.joints[i])) for i, name in enumerate(JOINT_NAMES)]
        self.publisher.publish(msg)

        # HUD display throttled to ~2 Hz
        if now - self.last_hud_print > 0.5:
            self.last_hud_print = now
            self.render_hud()

    def handle_keyboard_input(self, dt: float) -> None:
        speed_scale = 0.25 if self.precision_mode else 1.0
        v_lin = float(self.get_parameter("ik_linear_speed").value) * speed_scale * 3.0
        v_joint = float(self.get_parameter("joint_speed").value) * speed_scale * 2.0

        key = get_key_nonblocking()
        if key is None:
            return

        k = key.lower()
        if k == "m":
            self.cartesian_mode = not self.cartesian_mode
            name = "CARTESIAN IK" if self.cartesian_mode else "JOINT JOG"
            self.get_logger().info(f"🕹️ Mode Switched: {name}")
        elif k == "p":
            self.precision_mode = not self.precision_mode
            name = "PRECISION (Slow)" if self.precision_mode else "NORMAL"
            self.get_logger().info(f"⚡ Speed: {name}")
        elif k == "h":
            self.joints = [float(v) for v in POSES["ready"]]
            self.ee_x, self.ee_y, self.ee_z = 0.22, 0.00, 0.08
            self.wrist_rot = 90.0
            self.get_logger().info("🏠 Reset Arm to READY pose")
        elif k == "y":
            self.print_calibration_report()
        elif k == "o":
            self.joints[5] = float(GRIPPER_OPEN)
            self.get_logger().info(f"👐 Gripper OPEN ({GRIPPER_OPEN}°)")
        elif k == "c":
            self.joints[5] = float(GRIPPER_CLOSED)
            self.get_logger().info(f"✊ Gripper CLOSED ({GRIPPER_CLOSED}°)")
        elif k == "[":
            self.joints[5] = clamp_degrees("gripper", self.joints[5] - 1.0)
        elif k == "]":
            self.joints[5] = clamp_degrees("gripper", self.joints[5] + 1.0)
        elif k == "?":
            self.print_keyboard_help()

        # Motion keys
        if self.cartesian_mode:
            dx = dy = dz = dw = 0.0
            if k == "w":
                dx += v_lin * dt
            elif k == "s":
                dx -= v_lin * dt
            elif k == "a":
                dy += v_lin * dt
            elif k == "d":
                dy -= v_lin * dt
            elif k == "r":
                dz += v_lin * dt
            elif k == "f":
                dz -= v_lin * dt
            elif k == "q":
                dw -= 45.0 * dt
            elif k == "e":
                dw += 45.0 * dt

            if dx or dy or dz or dw:
                nx = self.ee_x + dx
                ny = self.ee_y + dy
                nz = max(0.005, self.ee_z + dz)
                nw = clamp_degrees("wrist_rotation", self.wrist_rot + dw)
                sol = solve_ik(nx, ny, nz, gripper=int(self.joints[5]), wrist_rotation=nw)
                if sol is not None:
                    self.ee_x, self.ee_y, self.ee_z = nx, ny, nz
                    self.wrist_rot = float(nw)
                    for i in range(5):
                        self.joints[i] = float(sol[i])
        else:
            if k == "w":
                self.joints[1] = clamp_degrees("shoulder", self.joints[1] + v_joint * dt)
            elif k == "s":
                self.joints[1] = clamp_degrees("shoulder", self.joints[1] - v_joint * dt)
            elif k == "a":
                self.joints[0] = clamp_degrees("base", self.joints[0] + v_joint * dt)
            elif k == "d":
                self.joints[0] = clamp_degrees("base", self.joints[0] - v_joint * dt)
            elif k == "r":
                self.joints[2] = clamp_degrees("elbow", self.joints[2] + v_joint * dt)
            elif k == "f":
                self.joints[2] = clamp_degrees("elbow", self.joints[2] - v_joint * dt)
            elif k == "q":
                self.joints[4] = clamp_degrees("wrist_rotation", self.joints[4] - v_joint * dt)
            elif k == "e":
                self.joints[4] = clamp_degrees("wrist_rotation", self.joints[4] + v_joint * dt)

    def handle_joystick_input(self, dt: float) -> None:
        if self.latest_joy is None:
            return

        axes = self.latest_joy.axes
        buttons = self.latest_joy.buttons
        is_ucom = (self.controller_type == "ucom")
        speed_scale = 0.25 if self.precision_mode else 1.0

        # Gripper buttons:
        # Ucom: R1 = button 5, L1 = button 4
        # Xbox: RB = button 5, LB = button 4
        grip_speed = 25.0 * dt * speed_scale
        if len(buttons) > 5 and buttons[5]:
            self.joints[5] = clamp_degrees("gripper", self.joints[5] + grip_speed)
        if len(buttons) > 4 and buttons[4]:
            self.joints[5] = clamp_degrees("gripper", self.joints[5] - grip_speed)

        # Ucom D-pad (usually on axes 4 & 5 or 0 & 1 depending on 'analog' mode)
        if len(axes) > 5 and abs(axes[5]) > 0.5:
            self.joints[5] = clamp_degrees("gripper", self.joints[5] + axes[5] * 15.0 * dt)

        if self.cartesian_mode:
            lx = self.apply_deadzone(axes[0] if len(axes) > 0 else 0.0)
            ly = self.apply_deadzone(axes[1] if len(axes) > 1 else 0.0)
            # Ucom right stick vertical is typically axis 3 or axis 2
            rz_axis = 3 if len(axes) > 3 else (2 if len(axes) > 2 else -1)
            rw_axis = 2 if len(axes) > 2 else -1
            rz = self.apply_deadzone(axes[rz_axis]) if rz_axis >= 0 else 0.0
            rw = self.apply_deadzone(axes[rw_axis]) if rw_axis >= 0 and rw_axis != rz_axis else 0.0

            v_lin = float(self.get_parameter("ik_linear_speed").value) * speed_scale
            dx = ly * v_lin * dt
            dy = -lx * v_lin * dt
            dz = rz * v_lin * dt
            dw = rw * 45.0 * dt * speed_scale

            nx = self.ee_x + dx
            ny = self.ee_y + dy
            nz = max(0.005, self.ee_z + dz)
            nw = clamp_degrees("wrist_rotation", self.wrist_rot + dw)

            sol = solve_ik(nx, ny, nz, gripper=int(self.joints[5]), wrist_rotation=nw)
            if sol is not None:
                self.ee_x, self.ee_y, self.ee_z = nx, ny, nz
                self.wrist_rot = float(nw)
                for i in range(5):
                    self.joints[i] = float(sol[i])
        else:
            # Joint Jog
            v_joint = float(self.get_parameter("joint_speed").value) * speed_scale
            j_base = self.apply_deadzone(axes[0] if len(axes) > 0 else 0.0)
            j_shld = self.apply_deadzone(axes[1] if len(axes) > 1 else 0.0)
            rz_axis = 3 if len(axes) > 3 else 1
            j_elbw = self.apply_deadzone(axes[rz_axis]) if len(axes) > rz_axis else 0.0

            self.joints[0] = clamp_degrees("base", self.joints[0] - j_base * v_joint * dt)
            self.joints[1] = clamp_degrees("shoulder", self.joints[1] - j_shld * v_joint * dt)
            self.joints[2] = clamp_degrees("elbow", self.joints[2] - j_elbw * v_joint * dt)

            try:
                tip = forward_kinematics(self.joints[:5])
                self.ee_x, self.ee_y, self.ee_z = tip[0], tip[1], tip[2]
            except Exception:
                pass

    def render_hud(self) -> None:
        mode_tag = "IK-XYZ" if self.cartesian_mode else "JOINT "
        speed_tag = "SLOW" if self.precision_mode else "NORM"
        grip_val = int(round(self.joints[5]))
        bar_len = 10
        filled = int(round(((grip_val - 10) / 100.0) * bar_len))
        filled = max(0, min(bar_len, filled))
        grip_bar = "[" + "=" * filled + ">" + " " * (bar_len - filled) + "]"

        input_tag = "JOY" if self.input_type == "joystick" else "KEY"
        status = (
            f"[{input_tag}|{mode_tag}|{speed_tag}] "
            f"XYZ: ({self.ee_x:.3f}, {self.ee_y:.3f}, {self.ee_z:.3f})m | "
            f"GRIP: {grip_bar} {grip_val:3d}° | Press Y / 'y' to Calibrate"
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

    def destroy_node(self) -> bool:
        if self.orig_term_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_term_settings)
            except Exception:
                pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ManualControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
