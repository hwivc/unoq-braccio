"""Manual Control & Gripper Calibration Node for Arduino UNO Q Braccio.

Supports two input types via the 'input_type' parameter:
  1. 'joystick' : USB Gamepad / Joystick (supports Ucom / Microtik / DragonRise and Xbox)
  2. 'keyboard' : Direct interactive terminal keyboard teleoperation

Safety features:
  - DOES NOT jump or force the arm erect on startup.
  - Reads current arm position from /joint_states if available.
  - Waits for active user input before publishing any motion commands.
  - Configurable joystick device_id and device_name.
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
    GRIPPER_RAD_MAX,
    GRIPPER_RAD_MIN,
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


def urdf_rad_to_servo(name: str, rad: float) -> float:
    """Convert URDF radian position back to Braccio servo degrees."""
    if name in ("shoulder", "elbow", "wrist_vertical"):
        return 180.0 - math.degrees(rad)
    if name in ("base", "wrist_rotation"):
        return 90.0 + math.degrees(rad)
    if name == "gripper":
        span = GRIPPER_RAD_MAX - GRIPPER_RAD_MIN
        frac = (rad - GRIPPER_RAD_MIN) / span if span > 1e-6 else 0.0
        return 10.0 + max(0.0, min(1.0, frac)) * 100.0
    return 90.0


class ManualControl(Node):
    def __init__(self) -> None:
        super().__init__("manual_control")

        # Configurable Parameters
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
        self.cmd_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        self.joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value

        self.publisher = self.create_publisher(JointState, self.cmd_topic, 10)

        # State Variables
        self.joints = [float(v) for v in POSES["ready"]]
        self.cartesian_mode = True
        self.precision_mode = False

        self.ee_x = 0.22
        self.ee_y = 0.00
        self.ee_z = 0.08
        self.wrist_rot = 90.0

        # Safety & connection flags
        self.active_control = False  # Only send commands once user actually inputs
        self.first_joy_received = False
        self.latest_joy = None
        self.last_buttons = []
        self.last_loop_time = time.monotonic()
        self.last_hud_print = 0.0
        self.last_joy_warn = 0.0

        # Listen to existing robot joint state so we don't jump on launch
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.on_joint_states, 10
        )

        # Keyboard vs Joystick initialization
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
            self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.on_joy, 10)
            self.print_joystick_help()

        # Timer loop for control
        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(period, self.control_loop)

    def on_joint_states(self, msg: JointState) -> None:
        """Initialize joints to the robot's current physical/simulated pose if we haven't started moving yet."""
        if self.active_control:
            return  # User has taken manual control; do not overwrite from feedback
        
        name_map = dict(zip(msg.name, msg.position))
        updated = False
        for i, jname in enumerate(JOINT_NAMES):
            if jname in name_map:
                deg = urdf_rad_to_servo(jname, name_map[jname])
                self.joints[i] = float(clamp_degrees(jname, deg))
                updated = True
        
        if updated:
            try:
                tip = forward_kinematics(self.joints[:5])
                self.ee_x, self.ee_y, self.ee_z = tip[0], tip[1], tip[2]
                self.wrist_rot = self.joints[4]
            except Exception:
                pass

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
    M     : Toggle Mode (Cartesian IK <-> Joint Jogging)
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
    {'R1 (Btn 6)' if is_ucom else 'RB'} : Step Gripper CLOSE (+1 deg)
    {'L1 (Btn 5)' if is_ucom else 'LB'} : Step Gripper OPEN (-1 deg)
    {'Btn 3 (X)' if is_ucom else 'Btn A'} : Snap OPEN (10 deg)
    {'Btn 2 (O)' if is_ucom else 'Btn B'} : Snap CLOSED (95 deg)
    {'Btn 1 (▲)' if is_ucom else 'Btn Y'} : 📋 PRINT FULL CALIBRATION REPORT

  [ARM POSITIONING]
    Left Stick   : Move X (Forward/Back) & Y (Left/Right)
    Right Stick  : Vertical moves Z (Elevation); Horizontal rotates Wrist
    {'Select (Btn 9)' if is_ucom else 'Back/View'} : Toggle Mode (Cartesian IK <-> Joint Jog)
    {'Start (Btn 10)' if is_ucom else 'Start'}   : Toggle Speed (Normal <-> Precision)
    {'Btn 4 (■)' if is_ucom else 'Btn X'} : Reset to READY pose

  💡 Tip: On Ucom/Microtik gamepads, ensure the red 'ANALOG' LED is ON!
======================================================================
"""
        print(help_text)

    def on_joy(self, msg: Joy) -> None:
        if not self.first_joy_received:
            self.first_joy_received = True
            self.get_logger().info(
                f"✅ Connected to Joystick! Found {len(msg.axes)} axes and {len(msg.buttons)} buttons."
            )
            self.get_logger().info("💡 Move any thumbstick or press any button to begin controlling.")

        self.latest_joy = msg

        # Check for button edge presses
        if not self.last_buttons or len(self.last_buttons) != len(msg.buttons):
            self.last_buttons = list(msg.buttons)
            return

        def pressed(btn: int) -> bool:
            return bool(btn < len(msg.buttons) and msg.buttons[btn] and not self.last_buttons[btn])

        is_ucom = (self.controller_type == "ucom")

        # Mode switch: Ucom button 8 (Select) or Xbox button 6 (Back)
        mode_btn = 8 if is_ucom else 6
        if pressed(mode_btn):
            self.active_control = True
            self.cartesian_mode = not self.cartesian_mode
            name = "CARTESIAN IK (XYZ)" if self.cartesian_mode else "JOINT JOGGING"
            self.get_logger().info(f"🕹️ Mode Switched: {name}")

        # Speed switch: Ucom button 9 (Start) or Xbox button 7 (Start)
        speed_btn = 9 if is_ucom else 7
        if pressed(speed_btn):
            self.active_control = True
            self.precision_mode = not self.precision_mode
            name = "PRECISION (Slow)" if self.precision_mode else "NORMAL"
            self.get_logger().info(f"⚡ Speed: {name}")

        # Snap Open: Ucom button 2 (X) or Xbox button 0 (A)
        snap_open_btn = 2 if is_ucom else 0
        if pressed(snap_open_btn):
            self.active_control = True
            self.joints[5] = float(GRIPPER_OPEN)
            self.get_logger().info(f"👐 Gripper OPEN ({GRIPPER_OPEN}°)")

        # Snap Closed: Ucom button 1 (Circle) or Xbox button 1 (B)
        snap_closed_btn = 1 if is_ucom else 1
        if pressed(snap_closed_btn):
            self.active_control = True
            self.joints[5] = float(GRIPPER_CLOSED)
            self.get_logger().info(f"✊ Gripper CLOSED ({GRIPPER_CLOSED}°)")

        # Calibration Report: Ucom button 0 (Triangle) or Xbox button 3 (Y)
        calib_btn = 0 if is_ucom else 3
        if pressed(calib_btn):
            self.print_calibration_report()

        # Reset Ready: Ucom button 3 (Square) or Xbox button 2 (X)
        reset_btn = 3 if is_ucom else 2
        if pressed(reset_btn):
            self.active_control = True
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
            if not self.first_joy_received:
                if now - self.last_joy_warn > 3.0:
                    self.last_joy_warn = now
                    self.get_logger().warn(
                        f"⏳ Waiting for joystick data on topic '{self.joy_topic}'...\n"
                        "   Ensure joy_node is running or launch with: "
                        "ros2 launch unoq_braccio_bringup manual_control.launch.py device_id:=<X>"
                    )
                return
            self.handle_joystick_input(dt)

        # Only publish command if active control has been initiated by user
        if not self.active_control:
            return

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

        self.active_control = True
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
        speed_scale = 0.25 if self.precision_mode else 1.0

        # Gripper buttons:
        # Ucom: R1 = button 5, L1 = button 4
        # Xbox: RB = button 5, LB = button 4
        grip_speed = 30.0 * dt * speed_scale
        if len(buttons) > 5 and buttons[5]:
            self.active_control = True
            self.joints[5] = clamp_degrees("gripper", self.joints[5] + grip_speed)
        if len(buttons) > 4 and buttons[4]:
            self.active_control = True
            self.joints[5] = clamp_degrees("gripper", self.joints[5] - grip_speed)

        # Check axes motion
        lx = self.apply_deadzone(axes[0] if len(axes) > 0 else 0.0)
        ly = self.apply_deadzone(axes[1] if len(axes) > 1 else 0.0)
        
        # On Ucom / generic pads, right stick vertical can be axis 2, 3 or 4
        rz = 0.0
        rw = 0.0
        if len(axes) > 3:
            rz = self.apply_deadzone(axes[3])
            rw = self.apply_deadzone(axes[2])
        elif len(axes) > 2:
            rz = self.apply_deadzone(axes[2])

        # D-pad fine adjustment (axes 4/5 or hat)
        if len(axes) > 5:
            dpad_y = self.apply_deadzone(axes[5])
            if abs(dpad_y) > 0.0:
                self.active_control = True
                self.joints[5] = clamp_degrees("gripper", self.joints[5] + dpad_y * 15.0 * dt)

        # Did any stick move?
        if any(abs(v) > 0.001 for v in (lx, ly, rz, rw)):
            self.active_control = True

        if not self.active_control:
            return

        if self.cartesian_mode:
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
                self.ee_x = nx
                self.ee_y = ny
                self.ee_z = nz
                self.wrist_rot = float(nw)
                for i in range(5):
                    self.joints[i] = float(sol[i])
        else:
            # Joint Jog
            v_joint = float(self.get_parameter("joint_speed").value) * speed_scale
            self.joints[0] = clamp_degrees("base", self.joints[0] - lx * v_joint * dt)
            self.joints[1] = clamp_degrees("shoulder", self.joints[1] - ly * v_joint * dt)
            self.joints[2] = clamp_degrees("elbow", self.joints[2] - rz * v_joint * dt)

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
