"""Serial protocol checks. Pure Python: run with ``python test_protocol.py``
or ``pytest``; no ROS, pyserial or board needed.

Covers: the bridge parses the firmware's STAT line, builds valid commands,
and the firmware's joint limits and start pose match braccio_model.py.
"""

import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from unoq_braccio_driver.braccio_model import (  # noqa: E402
    JOINT_LIMITS,
    JOINT_NAMES,
    POSES,
    START_POSE,
    command_line_from_positions,
)
from unoq_braccio_driver.braccio_protocol import parse_status, speed_command  # noqa: E402

FIRMWARE = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..",
    "firmware", "braccio_uno_firmware", "braccio_uno_firmware.ino",
)


def _firmware_array(name):
    source = open(FIRMWARE, encoding="utf-8").read()
    match = re.search(rf"{name}\[JOINTS\] = \{{([^}}]*)\}}", source)
    assert match, name
    return [int(v) for v in match.group(1).split(",")]


def test_parse_status_line():
    status = parse_status(
        "STAT pos=90,45,180,180,90,10 target=90,90,90,90,90,25 moving=1 "
        "speed=60 moves=3 uptime_ms=12345"
    )
    assert status["pos"] == [90, 45, 180, 180, 90, 10]
    assert status["target"] == [90, 90, 90, 90, 90, 25]
    assert status["moving"] is True
    assert status["speed"] == 60
    assert status["moves"] == 3


def test_parse_status_rejects_other_lines():
    for line in ("OK", "DONE", "ERR bad_move", "READY BRACCIO_UNO 1", "",
                 "STAT pos=1,2,3", "STAT pos=a,b,c,d,e,f", "STAT moving=1"):
        assert parse_status(line) is None, line


def test_move_command_is_clamped_and_complete():
    line = command_line_from_positions(["base", "gripper"], [200.0, 5.0])
    parts = line.split()
    assert parts[0] == "M" and len(parts) == 1 + len(JOINT_NAMES)
    assert int(parts[1]) == JOINT_LIMITS["base"].maximum
    assert int(parts[6]) == JOINT_LIMITS["gripper"].minimum


def test_speed_command_is_clamped():
    assert speed_command(60) == "V 60"
    assert speed_command(1) == "V 10"
    assert speed_command(999) == "V 180"


def test_firmware_matches_joint_model():
    assert _firmware_array("MIN_LIMITS") == [JOINT_LIMITS[n].minimum for n in JOINT_NAMES]
    assert _firmware_array("MAX_LIMITS") == [JOINT_LIMITS[n].maximum for n in JOINT_NAMES]
    assert _firmware_array("START_POSE") == START_POSE


def test_start_pose_is_standing_straight_up():
    assert START_POSE == POSES["ready"]
    assert START_POSE[:5] == [90, 90, 90, 90, 90]


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as exc:
                failed += 1
                print(f"FAIL {name}: {exc!r}")
    sys.exit(1 if failed else 0)
