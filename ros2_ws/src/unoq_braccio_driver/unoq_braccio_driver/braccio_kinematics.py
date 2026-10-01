"""Braccio kinematics shared by the simulator, IK helpers and pick/place demo.

Pure Python (no ROS, no numpy) so it can be unit-tested anywhere.

Servo convention (degrees, matches the firmware and ``braccio_model``):

* ``base``            90 faces forward (+x); larger values turn left (+y).
* ``shoulder``        90 is vertical; larger values lean the upper arm forward.
* ``elbow``           90 is straight; larger values fold the forearm forward/down.
* ``wrist_vertical``  90 is straight; larger values fold the gripper forward/down.
* ``wrist_rotation``  90 is centred.
* ``gripper``         10 is open, 110 is fully closed.

``ready`` (90, 90, 90, 90, 90) is therefore the arm standing straight up.
The URDF joint zeros are not the servo zeros, so the simulator converts with
:func:`servo_to_urdf`. ``URDF_CHAIN`` mirrors ``braccio.urdf.xacro`` and is used
by :func:`forward_kinematics` to check the IK against the real geometry.
"""

import math

# Geometry taken from braccio.urdf.xacro (metres).
BASE_HEIGHT = 0.03          # world -> braccio_base_link origin
SHOULDER_RADIAL = -0.002    # shoulder axis, radial offset from the base axis
SHOULDER_HEIGHT = 0.102     # shoulder axis height above the ground
UPPER_ARM = 0.125
FOREARM = 0.125
WRIST_LINK = 0.06           # wrist_vertical axis -> wrist_roll origin
# wrist_roll origin -> grasp point (the cube centre when gripping). The finger
# meshes reach 0.130 along the tool, so a cube centred at 0.117 leaves the
# fingertips ~2 mm above the table instead of driving them into it.
TIP_LENGTH = 0.117
TOOL_LENGTH = WRIST_LINK + TIP_LENGTH

GRIPPER_OPEN = 10
GRIPPER_CLOSED = 95         # pads touch a 30 mm cube just before 90; 95 squeezes it

# Gripper joint range in the URDF. The left finger uses the same range and
# the same angle as the right one: its joint frame is flipped 180 degrees about
# the finger axis (rpy pitch = pi + 0.2967) and its axis is reversed, so equal
# joint angles put the two fingertips at mirror-image positions and they close
# towards each other. An earlier "left = offset - right" mapping swung both
# fingers to the same side together, like windscreen wipers, instead of
# pinching.
GRIPPER_RAD_MIN = 0.1750
GRIPPER_RAD_MAX = 1.2741

URDF_ARM_JOINTS = ["base", "shoulder", "elbow", "wrist_vertical", "wrist_rotation"]
URDF_JOINT_NAMES = URDF_ARM_JOINTS + ["gripper", "left_gripper"]


def servo_to_urdf(name: str, value: float) -> float:
    """Servo degrees -> URDF joint position in radians."""
    value = float(value)
    if name in ("shoulder", "elbow", "wrist_vertical"):
        return math.radians(180.0 - value)
    if name in ("base", "wrist_rotation"):
        return math.radians(value - 90.0)
    if name == "gripper":
        span = GRIPPER_RAD_MAX - GRIPPER_RAD_MIN
        return GRIPPER_RAD_MIN + max(0.0, min(1.0, (value - 10.0) / 100.0)) * span
    if name == "left_gripper":
        return servo_to_urdf("gripper", value)
    raise KeyError(name)


def servo_positions_to_urdf(values_by_name: dict) -> list:
    """Full URDF joint vector (including the mirrored finger) for the six servos."""
    return [
        servo_to_urdf(
            name,
            values_by_name["gripper" if name == "left_gripper" else name],
        )
        for name in URDF_JOINT_NAMES
    ]


# --- forward kinematics straight from the URDF joint chain -----------------

def _rpy(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _axis_rotation(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


# (origin xyz, origin rpy, axis) per joint, from braccio.urdf.xacro.
URDF_CHAIN = [
    ((0.0, 0.0, 0.01), (0.0, 0.0, 0.0), (0, 0, 1)),                      # base
    ((0.0, -0.002, 0.072), (-math.pi / 2, 0.0, 0.0), (1, 0, 0)),         # shoulder
    ((0.0, 0.0, 0.125), (-math.pi / 2, 0.0, 0.0), (1, 0, 0)),            # elbow
    ((0.0, 0.0, 0.125), (-math.pi / 2, 0.0, 0.0), (1, 0, 0)),            # wrist_vertical
    ((0.0, 0.0, 0.06), (0.0, 0.0, math.pi / 2), (0, 0, -1)),             # wrist_rotation
]
# world -> base_link fixed joint (yaw so that servo base=90 faces +x).
URDF_BASE_ORIGIN = (0.0, 0.0, 0.02)
URDF_BASE_YAW = -math.pi / 2


def _tool_pose(servo_degrees):
    """World position and rotation of the wrist_roll frame for the five arm servos."""
    rot = _rpy(0.0, 0.0, URDF_BASE_YAW)
    pos = list(URDF_BASE_ORIGIN)
    for (origin, rpy, axis), name, degrees in zip(
        URDF_CHAIN, URDF_ARM_JOINTS, servo_degrees
    ):
        offset = _matvec(rot, origin)
        pos = [pos[i] + offset[i] for i in range(3)]
        rot = _matmul(_matmul(rot, _rpy(*rpy)), _axis_rotation(axis, servo_to_urdf(name, degrees)))
    return pos, rot


def forward_kinematics(servo_degrees, tip=(0.0, 0.0, TIP_LENGTH)):
    """World position of the fingertip for the five arm servos."""
    pos, rot = _tool_pose(servo_degrees)
    offset = _matvec(rot, tip)
    return [pos[i] + offset[i] for i in range(3)]


def grasp_wrist_rotation(servo_degrees, object_yaw_deg=0.0):
    """wrist_rotation that makes the fingers close square to an object's faces.

    The fingers close along the wrist_roll frame's x axis. With wrist_rotation
    fixed at 90 that axis follows the base angle, so a cube off to the side
    gets pinched across its diagonal and slips out. This picks the roll
    nearest 90 that lines the closing axis up with a face (any multiple of
    90 degrees from ``object_yaw_deg``, the cube's yaw in the world).
    """
    servo = list(servo_degrees[:4]) + [90.0]
    for _ in range(3):  # the tool is not exactly vertical: refine a couple of times
        _, rot = _tool_pose(servo)
        yaw = math.degrees(math.atan2(rot[1][0], rot[0][0]))
        error = (yaw - object_yaw_deg + 45.0) % 90.0 - 45.0
        servo[4] = max(0.0, min(180.0, servo[4] - error))
    return int(round(servo[4]))


# --- inverse kinematics -----------------------------------------------------

def _two_link(target_r: float, target_z: float, elbow_up: bool = True):
    """Planar solution. Returns (shoulder_elev, forearm_elev) or None."""
    dr = target_r - SHOULDER_RADIAL
    dz = target_z - SHOULDER_HEIGHT
    dist = math.hypot(dr, dz)
    if dist > UPPER_ARM + FOREARM - 1e-4 or dist < abs(UPPER_ARM - FOREARM) + 1e-4:
        return None
    cos_bend = (dist * dist - UPPER_ARM ** 2 - FOREARM ** 2) / (2.0 * UPPER_ARM * FOREARM)
    bend = math.acos(max(-1.0, min(1.0, cos_bend)))  # forearm relative to upper arm
    if not elbow_up:
        bend = -bend
    shoulder = math.atan2(dz, dr) + math.atan2(
        FOREARM * math.sin(bend), UPPER_ARM + FOREARM * math.cos(bend)
    )
    # elbow-up means the forearm points below the upper arm
    return shoulder, shoulder - bend


def planar_ik(reach_m, z_m, pitch_deg, near=None):
    """Shoulder, elbow and wrist_vertical servo degrees (floats) that put the
    fingertip at signed horizontal ``reach`` from the base axis and height
    ``z``, with the tool at ``pitch`` degrees (-90 points straight down, 0
    level, 90 straight up). Returns ``None`` when unreachable within limits.

    Without ``near`` the elbow-up solution is preferred; with ``near`` (current
    shoulder, elbow, wrist_vertical) the solution closest to it is returned,
    so continuous jogging never flips between elbow branches.
    """
    from unoq_braccio_driver.braccio_model import JOINT_LIMITS

    pitch = math.radians(pitch_deg)
    wrist_r = reach_m - TOOL_LENGTH * math.cos(pitch)
    wrist_z = z_m - TOOL_LENGTH * math.sin(pitch)
    candidates = []
    for elbow_up in (True, False):
        solution = _two_link(wrist_r, wrist_z, elbow_up)
        if solution is None:
            continue
        shoulder_elev, forearm_elev = solution
        servo = [
            180.0 - math.degrees(shoulder_elev),
            90.0 - math.degrees(forearm_elev - shoulder_elev),
            90.0 - math.degrees(pitch - forearm_elev),
        ]
        if all(
            JOINT_LIMITS[n].minimum <= v <= JOINT_LIMITS[n].maximum
            for n, v in zip(("shoulder", "elbow", "wrist_vertical"), servo)
        ):
            candidates.append(servo)
    if not candidates:
        return None
    if near is None:
        return candidates[0]
    return min(candidates, key=lambda c: max(abs(a - b) for a, b in zip(c, near)))


def tool_pitch(servo_degrees):
    """Tool pitch in degrees (as used by :func:`planar_ik`) for the arm servos."""
    return 360.0 - servo_degrees[1] - servo_degrees[2] - servo_degrees[3]


def solve_ik(x_m, y_m, z_m, gripper=GRIPPER_OPEN, wrist_rotation=90):
    """Servo degrees that put the fingertip at ``(x, y, z)`` (world, metres).

    The tool is kept as close to pointing straight down as the reach allows.
    Returns ``None`` when the point cannot be reached within servo limits.
    """
    from unoq_braccio_driver.braccio_model import JOINT_LIMITS

    radius = math.hypot(x_m, y_m)
    base = 90.0 + math.degrees(math.atan2(y_m, x_m))

    for pitch_deg in range(-90, -9, 5):
        pitch = math.radians(pitch_deg)
        wrist_r = radius - TOOL_LENGTH * math.cos(pitch)
        wrist_z = z_m - TOOL_LENGTH * math.sin(pitch)
        solution = _two_link(wrist_r, wrist_z)
        if solution is None:
            continue
        shoulder_elev, forearm_elev = solution
        servo = [
            base,
            180.0 - math.degrees(shoulder_elev),
            90.0 - math.degrees(forearm_elev - shoulder_elev),
            90.0 - math.degrees(pitch - forearm_elev),
            float(wrist_rotation),
        ]
        names = URDF_ARM_JOINTS
        if all(
            JOINT_LIMITS[n].minimum <= v <= JOINT_LIMITS[n].maximum
            for n, v in zip(names, servo)
        ):
            return [int(round(v)) for v in servo] + [int(gripper)]
    return None
