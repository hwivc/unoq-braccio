"""Manual Control & Gripper Calibration Node for Arduino UNO Q Braccio.

Supports two input types via the 'input_type' parameter:
  1. 'joystick' : USB Gamepad / Joystick (Ucom / Microtik / DragonRise or Xbox)
  2. 'keyboard' : Direct interactive terminal keyboard teleoperation

Two control modes (toggle with Select/Back or 'M'):
  * TOOL  : cylindrical tool control - base yaw, reach, height, tool pitch and
            wrist roll. The tool pitch is held while you move, so the gripper
            keeps its angle instead of snapping between IK solutions.
  * JOINT : drive each servo directly.

Smoothness and safety:
  - Joint targets are kept as floats; only the published command is rounded,
    so slow / precision motion never stalls on rounding.
  - The published command is slew-limited (max_joint_rate), so IK jumps near
    singularities (e.g. straight up) become smooth moves.
  - Does not move on startup: syncs to /joint_states if available and waits
    for input before publishing.
  - Stops if joystick messages stop arriving (unplugged pad / dead joy_node).
  - Commands are only published when they change (plus a 1 Hz keepalive).
"""

import math
import os
import sys
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
    planar_ik,
    tool_pitch,
)
from unoq_braccio_driver.braccio_model import JOINT_LIMITS, JOINT_NAMES, POSES

# Cross-platform single key reader for keyboard mode
if os.name == "nt":
    import msvcrt

    def get_key_nonblocking() -> str | None:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in (b"\x00", b"\xe0"):  # arrow / function key prefix
                msvcrt.getch()
                return None
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


# Joy index layouts as published by the ROS 2 `joy_node` (raw SDL joystick).
# Any entry can be overridden with the matching ROS parameter (axis_* / button_*).
PROFILES = {
    # Generic DragonRise "USB Gamepad" (Ucom / Microtik) with ANALOG on.
    "ucom": {
        "axis_lx": 0, "axis_ly": 1, "axis_rx": 2, "axis_ry": 3,
        "axis_dpad_x": 4, "axis_dpad_y": 5,
        "button_report": 0,    # Triangle / 1
        "button_close": 1,     # Circle / 2
        "button_open": 2,      # Cross / 3
        "button_home": 3,      # Square / 4
        "button_grip_open": 4,   # L1
        "button_grip_close": 5,  # R1
        "button_mode": 8,      # Select
        "button_speed": 9,     # Start
    },
    # Xbox 360 / One pad. Axes 2 and 5 are the triggers (rest at +1.0) and
    # must not be read as sticks.
    "xbox": {
        "axis_lx": 0, "axis_ly": 1, "axis_rx": 3, "axis_ry": 4,
        "axis_dpad_x": 6, "axis_dpad_y": 7,
        "button_open": 0,      # A
        "button_close": 1,     # B
        "button_home": 2,      # X
        "button_report": 3,    # Y
        "button_grip_open": 4,   # LB
        "button_grip_close": 5,  # RB
        "button_mode": 6,      # Back / View
        "button_speed": 7,     # Start / Menu
    },
}

ARM = ("base", "shoulder", "elbow", "wrist_vertical", "wrist_rotation")
GRIP = 5


def clamp(name: str, value: float) -> float:
    limit = JOINT_LIMITS[name]
    return max(float(limit.minimum), min(float(limit.maximum), float(value)))


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

        self.declare_parameter("input_type", "joystick")  # 'joystick' or 'keyboard'
        self.declare_parameter("controller_type", "ucom")  # 'ucom' or 'xbox'
        self.declare_parameter("command_topic", "/braccio/joint_command")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("publish_rate", 30.0)  # Hz
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("joy_timeout", 0.5)  # s without /joy -> stop
        # Full-stick speeds (precision mode multiplies them by precision_scale).
        self.declare_parameter("linear_speed", 0.10)  # m/s (reach, height)
        self.declare_parameter("yaw_speed", 60.0)  # deg/s (base)
        self.declare_parameter("pitch_speed", 60.0)  # deg/s (tool pitch)
        self.declare_parameter("joint_speed", 60.0)  # deg/s (joint mode, wrist roll)
        self.declare_parameter("gripper_speed", 60.0)  # deg/s
        self.declare_parameter("precision_scale", 0.25)
        self.declare_parameter("max_joint_rate", 120.0)  # deg/s slew limit on output
        # Keyboard: one key press (or OS auto-repeat) = one step.
        self.declare_parameter("key_step_m", 0.005)
        self.declare_parameter("key_step_deg", 3.0)

        self.input_type = self.get_parameter("input_type").value.lower()
        self.controller_type = self.get_parameter("controller_type").value.lower()
        if self.controller_type not in PROFILES:
            self.get_logger().warn(f"Unknown controller_type '{self.controller_type}', using 'ucom'")
            self.controller_type = "ucom"
        self.map = {}
        for key, default in PROFILES[self.controller_type].items():
            self.declare_parameter(key, -1)
            value = int(self.get_parameter(key).value)
            self.map[key] = value if value >= 0 else default

        self.cmd_topic = self.get_parameter("command_topic").value
        self.joy_topic = self.get_parameter("joy_topic").value
        self.publisher = self.create_publisher(JointState, self.cmd_topic, 10)

        # target: where the user wants the arm (float servo degrees).
        # command: slew-limited version of target that is actually published.
        self.target = [float(v) for v in POSES["ready"]]
        self.command = list(self.target)
        self.tool_mode = True
        self.precision_mode = False
        self.sync_tool_from_joints()

        self.active_control = False  # nothing is published until the user acts
        self.latest_joy = None
        self.last_joy_time = 0.0
        self.last_buttons = []
        self.last_loop_time = time.monotonic()
        self.last_published = None
        self.last_publish_time = 0.0
        self.last_hud = None
        self.last_hud_time = 0.0
        self.last_wait_warn = 0.0
        self.blocked_warned = False

        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)

        self.orig_term_settings = None
        if self.input_type == "keyboard":
            if os.name != "nt":
                if not sys.stdin.isatty():
                    self.get_logger().error(
                        "Keyboard mode needs an interactive terminal. Run it with:\n"
                        "  ros2 run unoq_braccio_driver manual_control --ros-args -p input_type:=keyboard"
                    )
                else:
                    self.orig_term_settings = termios.tcgetattr(sys.stdin)
                    tty.setcbreak(sys.stdin.fileno())
            self.print_keyboard_help()
        else:
            self.create_subscription(Joy, self.joy_topic, self.on_joy, 10)
            self.print_joystick_help()

        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(period, self.control_loop)

    # ------------------------------------------------------------------ state

    def sync_tool_from_joints(self) -> None:
        """Recompute the tool-space state (yaw, reach, z, pitch) from target joints."""
        tip = forward_kinematics(self.target[:5])
        yaw = math.radians(self.target[0] - 90.0)
        self.reach = tip[0] * math.cos(yaw) + tip[1] * math.sin(yaw)
        self.z = tip[2]
        self.pitch = tool_pitch(self.target)

    def on_joint_states(self, msg: JointState) -> None:
        """Follow the real/simulated arm until the user takes control, so we never jump."""
        if self.active_control:
            return
        positions = dict(zip(msg.name, msg.position))
        if not any(name in positions for name in JOINT_NAMES):
            return
        for i, name in enumerate(JOINT_NAMES):
            if name in positions:
                self.target[i] = clamp(name, urdf_rad_to_servo(name, positions[name]))
        self.command = list(self.target)
        self.sync_tool_from_joints()

    def go_home(self) -> None:
        self.target[:5] = [float(v) for v in POSES["ready"][:5]]
        self.sync_tool_from_joints()
        self.get_logger().info("🏠 Moving to READY pose")

    def set_gripper(self, value: float) -> None:
        self.target[GRIP] = clamp("gripper", value)

    def toggle_mode(self) -> None:
        self.tool_mode = not self.tool_mode
        self.sync_tool_from_joints()
        name = "TOOL (yaw / reach / height / pitch)" if self.tool_mode else "JOINT"
        self.get_logger().info(f"🕹️ Mode: {name}")

    def toggle_speed(self) -> None:
        self.precision_mode = not self.precision_mode
        self.get_logger().info(f"⚡ Speed: {'PRECISION (slow)' if self.precision_mode else 'NORMAL'}")

    # ---------------------------------------------------------------- motion

    def move_tool(self, d_yaw: float, d_reach: float, d_z: float, d_pitch: float, d_roll: float) -> None:
        """Apply a tool-space delta. Unreachable components are dropped one by one,
        so the arm slides along the edge of the workspace instead of freezing."""
        self.target[0] = clamp("base", self.target[0] + d_yaw)
        self.target[4] = clamp("wrist_rotation", self.target[4] + d_roll)
        if not (d_reach or d_z or d_pitch):
            return

        def attempt(reach, z, pitch):
            if abs(pitch) > 90.0:
                return False
            arm = planar_ik(reach, max(0.005, z), pitch, near=self.target[1:4])
            # A big jump means the IK flipped elbow branch: refuse it.
            if arm is None or max(abs(a - b) for a, b in zip(arm, self.target[1:4])) > 30.0:
                return False
            self.reach, self.z, self.pitch = reach, max(0.005, z), pitch
            self.target[1:4] = arm
            return True

        r, z, p = self.reach, self.z, self.pitch
        if attempt(r + d_reach, z + d_z, p + d_pitch):
            self.blocked_warned = False
            return
        if (d_reach or d_z) and not d_pitch:
            # Position matters more than holding the exact tool angle: let the
            # pitch give way a little rather than stopping.
            for offset in (o * s for o in range(2, 31, 2) for s in (1, -1)):
                if attempt(r + d_reach, z + d_z, p + offset):
                    self.blocked_warned = False
                    return
        moved = False
        for dr, dz, dp in ((d_reach, 0, 0), (0, d_z, 0), (0, 0, d_pitch)):
            if dr or dz or dp:
                moved |= attempt(self.reach + dr, self.z + dz, self.pitch + dp)
        if not moved and not self.blocked_warned:
            self.blocked_warned = True
            self.get_logger().warn(
                "⛔ Edge of workspace - try changing tool pitch (D-pad / T,G) or switch to JOINT mode"
            )

    def move_joints(self, deltas: dict) -> None:
        for name, delta in deltas.items():
            i = JOINT_NAMES.index(name)
            self.target[i] = clamp(name, self.target[i] + delta)
        self.sync_tool_from_joints()

    # ----------------------------------------------------------------- input

    def shaped_axis(self, axes, key: str) -> float:
        index = self.map[key]
        if index < 0 or index >= len(axes):
            return 0.0
        value = float(axes[index])
        dz = float(self.get_parameter("deadzone").value)
        if abs(value) < dz:
            return 0.0
        a = min(1.0, (abs(value) - dz) / (1.0 - dz))
        # Gentle near the centre for fine positioning, full speed at the edge.
        return math.copysign(0.3 * a + 0.7 * a ** 3, value)

    def button(self, buttons, key: str) -> bool:
        index = self.map[key]
        return 0 <= index < len(buttons) and bool(buttons[index])

    def on_joy(self, msg: Joy) -> None:
        if self.latest_joy is None:
            self.get_logger().info(
                f"✅ Joystick connected: {len(msg.axes)} axes, {len(msg.buttons)} buttons. "
                "Move a stick or press a button to take control."
            )
        self.latest_joy = msg
        self.last_joy_time = time.monotonic()

        previous = self.last_buttons if len(self.last_buttons) == len(msg.buttons) else list(msg.buttons)
        self.last_buttons = list(msg.buttons)

        def pressed(key: str) -> bool:
            index = self.map[key]
            return 0 <= index < len(msg.buttons) and bool(msg.buttons[index]) and not previous[index]

        if pressed("button_mode"):
            self.active_control = True
            self.toggle_mode()
        if pressed("button_speed"):
            self.toggle_speed()
        if pressed("button_open"):
            self.active_control = True
            self.set_gripper(GRIPPER_OPEN)
            self.get_logger().info(f"👐 Gripper OPEN ({GRIPPER_OPEN}°)")
        if pressed("button_close"):
            self.active_control = True
            self.set_gripper(GRIPPER_CLOSED)
            self.get_logger().info(f"✊ Gripper CLOSED ({GRIPPER_CLOSED}°)")
        if pressed("button_home"):
            self.active_control = True
            self.go_home()
        if pressed("button_report"):
            self.print_calibration_report()

    def handle_joystick(self, dt: float, now: float) -> None:
        if self.latest_joy is None:
            if now - self.last_wait_warn > 3.0:
                self.last_wait_warn = now
                self.get_logger().warn(
                    f"⏳ Waiting for joystick data on '{self.joy_topic}'. Is joy_node running? "
                    "(ros2 launch unoq_braccio_bringup manual_control.launch.py device_id:=<N>)"
                )
            return
        if now - self.last_joy_time > float(self.get_parameter("joy_timeout").value):
            return  # stale input: hold position

        axes = self.latest_joy.axes
        buttons = self.latest_joy.buttons
        scale = float(self.get_parameter("precision_scale").value) if self.precision_mode else 1.0

        lx = self.shaped_axis(axes, "axis_lx")
        ly = self.shaped_axis(axes, "axis_ly")
        rx = self.shaped_axis(axes, "axis_rx")
        ry = self.shaped_axis(axes, "axis_ry")
        dpad_y = self.shaped_axis(axes, "axis_dpad_y")
        grip = float(self.button(buttons, "button_grip_close")) - float(self.button(buttons, "button_grip_open"))

        if not any((lx, ly, rx, ry, dpad_y, grip)):
            return
        self.active_control = True

        if grip:
            g = float(self.get_parameter("gripper_speed").value) * scale * dt
            self.set_gripper(self.target[GRIP] + grip * g)

        joint = float(self.get_parameter("joint_speed").value) * scale * dt
        if self.tool_mode:
            lin = float(self.get_parameter("linear_speed").value) * scale * dt
            yaw = float(self.get_parameter("yaw_speed").value) * scale * dt
            pitch = float(self.get_parameter("pitch_speed").value) * scale * dt
            # ROS joy convention: stick left / up = +1.
            self.move_tool(
                d_yaw=lx * yaw,
                d_reach=ly * lin,
                d_z=ry * lin,
                d_pitch=dpad_y * pitch,
                d_roll=-rx * joint,
            )
        else:
            self.move_joints({
                "base": lx * joint,
                "shoulder": ly * joint,
                "elbow": ry * joint,
                "wrist_vertical": dpad_y * joint,
                "wrist_rotation": -rx * joint,
            })

    def handle_keyboard(self) -> None:
        keys = []
        while (key := get_key_nonblocking()) is not None:
            keys.append(key.lower())
        if not keys:
            return
        self.active_control = True

        scale = float(self.get_parameter("precision_scale").value) if self.precision_mode else 1.0
        lin = float(self.get_parameter("key_step_m").value) * scale
        deg = float(self.get_parameter("key_step_deg").value) * scale

        for k in keys:
            if k == "m":
                self.toggle_mode()
            elif k == "p":
                self.toggle_speed()
            elif k == "h":
                self.go_home()
            elif k == "y":
                self.print_calibration_report()
            elif k == "o":
                self.set_gripper(GRIPPER_OPEN)
                self.get_logger().info(f"👐 Gripper OPEN ({GRIPPER_OPEN}°)")
            elif k == "c":
                self.set_gripper(GRIPPER_CLOSED)
                self.get_logger().info(f"✊ Gripper CLOSED ({GRIPPER_CLOSED}°)")
            elif k == "[":
                self.set_gripper(self.target[GRIP] - max(1.0, deg / 3.0))
            elif k == "]":
                self.set_gripper(self.target[GRIP] + max(1.0, deg / 3.0))
            elif k == "?":
                self.print_keyboard_help()
            elif k in "wsadrftgqe":
                sign = 1.0 if k in "wartq" else -1.0
                axis = {"w": 0, "s": 0, "a": 1, "d": 1, "r": 2, "f": 2, "t": 3, "g": 3, "q": 4, "e": 4}[k]
                if self.tool_mode:
                    d = [0.0] * 5  # reach, yaw, z, pitch, roll
                    d[axis] = sign * (lin if axis in (0, 2) else deg)
                    self.move_tool(d_yaw=d[1], d_reach=d[0], d_z=d[2], d_pitch=d[3], d_roll=-d[4])
                else:
                    name = ("shoulder", "base", "elbow", "wrist_vertical", "wrist_rotation")[axis]
                    self.move_joints({name: sign * (-deg if axis == 4 else deg)})

    # ------------------------------------------------------------------ loop

    def control_loop(self) -> None:
        now = time.monotonic()
        dt = min(0.1, max(0.001, now - self.last_loop_time))
        self.last_loop_time = now

        if self.input_type == "keyboard":
            self.handle_keyboard()
        else:
            self.handle_joystick(dt, now)

        if not self.active_control:
            return

        # Slew-limit the published command towards the target.
        step = float(self.get_parameter("max_joint_rate").value) * dt
        for i in range(len(JOINT_NAMES)):
            delta = self.target[i] - self.command[i]
            self.command[i] += max(-step, min(step, delta))

        rounded = [int(round(clamp(name, self.command[i]))) for i, name in enumerate(JOINT_NAMES)]
        if rounded != self.last_published or now - self.last_publish_time > 1.0:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(JOINT_NAMES)
            msg.position = [float(v) for v in rounded]
            self.publisher.publish(msg)
            self.last_published = rounded
            self.last_publish_time = now

        if rounded != self.last_hud and now - self.last_hud_time > 0.5:
            self.last_hud = rounded
            self.last_hud_time = now
            self.render_hud(rounded)

    # -------------------------------------------------------------- display

    def render_hud(self, servo) -> None:
        mode = "TOOL " if self.tool_mode else "JOINT"
        speed = "SLOW" if self.precision_mode else "NORM"
        grip = servo[GRIP]
        filled = max(0, min(10, int(round((grip - 10) / 10.0))))
        bar = "[" + "#" * filled + "." * (10 - filled) + "]"
        joints = " ".join(f"{n[:5]}={v:3d}" for n, v in zip(ARM, servo))
        self.get_logger().info(
            f"[{mode}|{speed}] reach={self.reach:+.3f} z={self.z:.3f} pitch={self.pitch:+4.0f}° | "
            f"{joints} | grip {bar} {grip:3d}°"
        )

    def print_keyboard_help(self) -> None:
        print("""
======================================================================
⌨️  BRACCIO MANUAL CONTROL: KEYBOARD
======================================================================
              TOOL mode               JOINT mode
    W / S   : reach out / in          shoulder + / -
    A / D   : turn left / right       base + / -
    R / F   : up / down               elbow + / -
    T / G   : tilt tool up / down     wrist_vertical + / -
    Q / E   : roll wrist CCW / CW     wrist_rotation - / +

    [ / ]   : gripper open / close (small steps)
    O / C   : gripper snap OPEN / CLOSED
    M       : toggle TOOL <-> JOINT mode
    P       : toggle precision (slow) speed
    H       : go to READY pose
    Y       : print calibration report
    ?       : show this help        Ctrl+C: exit
======================================================================
""")

    def print_joystick_help(self) -> None:
        ucom = self.controller_type == "ucom"
        print(f"""
======================================================================
🎮 BRACCIO MANUAL CONTROL: {'UCOM / USB GAMEPAD' if ucom else 'XBOX'}
======================================================================
                     TOOL mode              JOINT mode
  Left stick  X  : turn base              base
  Left stick  Y  : reach out / in         shoulder
  Right stick Y  : up / down              elbow
  Right stick X  : roll wrist             wrist_rotation
  D-pad up/down  : tilt tool              wrist_vertical

  {'L1 / R1' if ucom else 'LB / RB'}        : gripper open / close (hold)
  {'Cross (3)' if ucom else 'A'}{' ' * (15 - len('Cross (3)' if ucom else 'A'))}: gripper snap OPEN ({GRIPPER_OPEN}°)
  {'Circle (2)' if ucom else 'B'}{' ' * (15 - len('Circle (2)' if ucom else 'B'))}: gripper snap CLOSED ({GRIPPER_CLOSED}°)
  {'Square (4)' if ucom else 'X'}{' ' * (15 - len('Square (4)' if ucom else 'X'))}: go to READY pose
  {'Triangle (1)' if ucom else 'Y'}{' ' * (15 - len('Triangle (1)' if ucom else 'Y'))}: print calibration report
  {'Select' if ucom else 'Back'}{' ' * (15 - len('Select' if ucom else 'Back'))}: toggle TOOL <-> JOINT mode
  Start          : toggle precision (slow) speed
{"  💡 Ucom / Microtik pads: press ANALOG so the red LED is ON." if ucom else ""}
  Wrong stick/button? Override any index, e.g. -p axis_rx:=2 -p button_mode:=8
  (check indexes with: ros2 topic echo /joy)
======================================================================
""")

    def print_calibration_report(self) -> None:
        grip_deg = int(round(self.target[GRIP]))
        tip = forward_kinematics(self.target[:5])
        angles = ", ".join(f"{name}={int(round(self.target[i]))}" for i, name in enumerate(JOINT_NAMES))

        print("\n" + "=" * 65)
        print("🎯 BRACCIO GRIPPER & POSE CALIBRATION REPORT")
        print("=" * 65)
        print(f"Current Gripper Angle : {grip_deg}° (Range: 10° open -> 110° fully shut)")
        print(f"Fingertip XYZ (World) : X = {tip[0]:.4f} m, Y = {tip[1]:.4f} m, Z = {tip[2]:.4f} m")
        print(f"Tool pitch            : {self.pitch:.1f}° (-90 = pointing straight down)")
        print(f"Full Joint Vector     : [{angles}]")
        print("-" * 65)
        print("📌 COPY & PASTE INTO CODE TO CALIBRATE:")
        print("  In 'ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_kinematics.py':")
        print(f"      GRIPPER_CLOSED = {grip_deg}")
        print("\n  In 'ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_workspace.py':")
        print(f"      CUBE_CENTRE_Z = {round(tip[2], 3)}")
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
