from dataclasses import dataclass

JOINT_NAMES = [
    "base",
    "shoulder",
    "elbow",
    "wrist_vertical",
    "wrist_rotation",
    "gripper",
]


@dataclass(frozen=True)
class JointLimit:
    minimum: int
    maximum: int


JOINT_LIMITS = {
    "base": JointLimit(0, 180),
    "shoulder": JointLimit(15, 165),
    "elbow": JointLimit(0, 180),
    "wrist_vertical": JointLimit(0, 180),
    "wrist_rotation": JointLimit(0, 180),
    "gripper": JointLimit(10, 110),
}

POSES = {
    "rest": [90, 45, 180, 180, 90, 10],
    "ready": [90, 90, 90, 90, 90, 25],
    "grip_test": [90, 90, 90, 90, 90, 95],
    "grip_full": [90, 90, 90, 90, 90, 110],
    "pickup": [90, 70, 45, 80, 90, 100],
    "drop": [130, 80, 60, 90, 90, 10],
    "wave": [60, 90, 90, 60, 120, 25],
}

# Power-on pose of the real arm: standing straight up, clear of the table.
# The UNO firmware starts here (START_POSE in braccio_uno_firmware.ino, checked
# by test_protocol.py), and the serial bridge reports it on /joint_states from
# launch until the board sends its real position, so RViz shows the same.
# The simulation also spawns in this pose.
START_POSE = POSES["ready"]


def clamp_degrees(name: str, value: float) -> int:
    limit = JOINT_LIMITS[name]
    return max(limit.minimum, min(limit.maximum, int(round(value))))


def command_line_from_positions(names: list[str], positions: list[float]) -> str:
    values_by_name = dict(zip(names, positions))
    degrees = [
        clamp_degrees(name, values_by_name.get(name, POSES["rest"][index]))
        for index, name in enumerate(JOINT_NAMES)
    ]
    return "M " + " ".join(str(value) for value in degrees)
