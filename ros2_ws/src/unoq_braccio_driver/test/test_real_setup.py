"""real_setup colour checks. Needs numpy and OpenCV, no ROS:
``python test_real_setup.py`` or ``pytest``.
"""

import os
import sys
import types

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
# real_setup imports ROS at module level; the colour functions do not need it.
for name in ("rclpy", "rclpy.node", "sensor_msgs", "sensor_msgs.msg"):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules["rclpy.node"].Node = getattr(sys.modules["rclpy.node"], "Node", object)
sys.modules["sensor_msgs.msg"].Image = getattr(sys.modules["sensor_msgs.msg"], "Image", object)
sys.modules["sensor_msgs.msg"].JointState = getattr(sys.modules["sensor_msgs.msg"], "JointState", object)

from unoq_braccio_driver.real_setup import find_cube, hue_name, measure_color  # noqa: E402


def _image(cube_hsv, at=(150, 100), size=40, shape=(240, 320)):
    """HSV image: white paper with one cube (and a black pen line)."""
    hsv = np.zeros(shape + (3,), np.uint8)
    hsv[:, :] = (0, 10, 240)               # white paper
    hsv[50, :] = (0, 0, 20)                # black pen line
    x, y = at
    hsv[y:y + size, x:x + size] = cube_hsv
    return hsv


def _in(ranges, h, s=200, v=180):
    return any(lo[0] <= h <= hi[0] and lo[1] <= s and lo[2] <= v for lo, hi in ranges)


def test_finds_the_cube_not_the_paper_or_pen():
    box = find_cube(_image((115, 200, 180)))
    assert box is not None and box[:4] == (150, 100, 190, 140)


def test_cube_cut_off_by_the_edge_is_ignored():
    assert find_cube(_image((115, 200, 180), at=(300, 100))) is None


def test_blue_and_red_learned_with_correct_ranges():
    hsv = _image((115, 200, 180))
    hue, _, ranges = measure_color(hsv, find_cube(hsv))
    assert hue_name(hue) == "blue" and _in(ranges, 115) and not _in(ranges, 0)

    hsv = _image((178, 210, 170))
    hsv[110:130, 160:180] = (2, 210, 170)   # red on both sides of the hue wrap
    hue, _, ranges = measure_color(hsv, find_cube(hsv))
    assert hue_name(hue) == "red"
    assert len(ranges) == 2 and _in(ranges, 178) and _in(ranges, 2) and not _in(ranges, 30)


def test_white_paper_never_matches_a_learned_colour():
    hsv = _image((28, 200, 200))
    _, _, ranges = measure_color(hsv, find_cube(hsv))
    assert not _in(ranges, 0, s=10, v=240)


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
