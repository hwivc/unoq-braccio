"""Text protocol of firmware/braccio_uno_firmware (pure Python, no ROS).

One line per command, 115200 baud:

    M <base> <shoulder> <elbow> <wrist_ver> <wrist_rot> <gripper>  -> OK, later DONE
    S                                                              -> STAT ...
    V <deg_per_s>                                                  -> OK
    H                                                              -> OK
    I                                                              -> READY ...

The firmware prints the READY banner once its servos are powered.
"""

from unoq_braccio_driver.braccio_model import JOINT_NAMES

READY_PREFIX = "READY"
STATUS_PREFIX = "STAT"
SPEED_MIN = 10
SPEED_MAX = 180


def speed_command(deg_per_s: float) -> str:
    """``V`` line, clamped to the speeds the firmware accepts."""
    return f"V {max(SPEED_MIN, min(SPEED_MAX, int(round(deg_per_s))))}"


def parse_status(line: str):
    """Parse a ``STAT`` line into a dict, or ``None`` if it is not one.

    ``pos`` and ``target`` become lists of servo degrees in ``JOINT_NAMES``
    order, ``moving`` a bool, other numeric fields ints.
    """
    parts = line.strip().split()
    if not parts or parts[0] != STATUS_PREFIX:
        return None
    status = {}
    for part in parts[1:]:
        key, sep, value = part.partition("=")
        if not sep:
            continue
        try:
            if key in ("pos", "target"):
                values = [int(v) for v in value.split(",")]
                if len(values) != len(JOINT_NAMES):
                    return None
                status[key] = values
            elif key == "moving":
                status[key] = value == "1"
            else:
                status[key] = int(value)
        except ValueError:
            return None
    if "pos" not in status:
        return None
    return status
