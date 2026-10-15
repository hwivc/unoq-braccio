"""Web teaching flow and cube colours, with a fake arm and camera. Needs numpy and
OpenCV, no ROS: ``python test_web_teach.py`` or ``pytest``.
"""

import os
import sys
import tempfile
import time
import types

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
# real_setup (used for measuring colours) imports ROS at module level; the colour maths does not need it.
for name in ("rclpy", "rclpy.node", "sensor_msgs", "sensor_msgs.msg"):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules["rclpy.node"].Node = getattr(sys.modules["rclpy.node"], "Node", object)
sys.modules["sensor_msgs.msg"].Image = getattr(sys.modules["sensor_msgs.msg"], "Image", object)
sys.modules["sensor_msgs.msg"].JointState = getattr(sys.modules["sensor_msgs.msg"], "JointState", object)

import cv2  # noqa: E402

from unoq_braccio_driver import braccio_workspace as ws  # noqa: E402
from unoq_braccio_driver import web_colors as wc  # noqa: E402
from unoq_braccio_driver.web_teach import Teacher  # noqa: E402


def _picture(hsv_colour=(115, 200, 200), at=(150, 100), size=30):
    """RGB picture of a grey table with one cube."""
    hsv = np.zeros((240, 320, 3), np.uint8)
    hsv[:, :] = (0, 10, 120)
    x, y = at
    hsv[y:y + size, x:x + size] = hsv_colour
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)


class FakeArm:
    """Arrives instantly wherever it is sent; the camera always shows one blue cube."""

    def __init__(self):
        self.servo = [90, 90, 90, 90, 90, 25]
        self.target = list(self.servo)
        self.frame = _picture()

    def send(self, pose):
        self.target = [int(round(v)) for v in pose]
        self.servo = list(self.target)
        return self.target

    def freeze(self):
        self.send(self.servo)

    def latest_frame(self):
        return self.frame

    def fresh_frames(self, count, timeout=6.0):
        return [self.frame] * count


def _wait(teacher, phase, timeout=20.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        state = teacher.state()
        if state["phase"] == phase and (state["choices"] or phase in ("ended",)):
            return state
        time.sleep(0.02)
    raise AssertionError(f"never reached {phase}; stuck at {teacher.state()['phase']}: {teacher.state()['prompt']}")


def _say(teacher, choice):
    ok, message = teacher.answer(choice)
    assert ok, message


def test_records_an_example_and_trains_like_teach_pick():
    ws.CUBE_HSV = {"blue": [((100, 130, 50), (130, 255, 255))]}
    with tempfile.TemporaryDirectory() as root:
        arm = FakeArm()
        teacher = Teacher(arm, "desk", root=root).start()
        _wait(teacher, "menu")
        assert teacher.answer("next")[0] is False           # not an option at the menu
        _say(teacher, "record")
        state = _wait(teacher, "above")
        assert state["reading"]["color"] == "blue"
        arm.send([100, 80, 120, 140, 95, ws.GRIPPER_OPEN])   # the operator drives the arm
        _say(teacher, "next")
        _wait(teacher, "grab")
        arm.send([100, 70, 140, 150, 95, ws.GRIPPER_OPEN])
        _say(teacher, "next")
        _wait(teacher, "close")
        arm.send([100, 75, 140, 150, 95, ws.GRIPPER_CLOSED])
        _say(teacher, "next")
        _wait(teacher, "check")
        _say(teacher, "yes")
        state = _wait(teacher, "menu")
        assert state["examples"] == 1 and state["model"]["chosen"] == "knn"
        assert any(e["kind"] == "saved" for e in state["events"])
        _say(teacher, "quit")
        _wait(teacher, "ended")

        saved = teacher.session.examples()
        assert len(saved) == 1
        example = saved[0]
        assert example["above"][:4] == [100, 80, 120, 140] and example["grab"][:4] == [100, 70, 140, 150]
        assert example["closed"] == ws.GRIPPER_CLOSED and example["color"] == "blue"
        assert os.path.exists(teacher.session.path("reference.jpg"))
        assert teacher.session.load_model() is not None      # saved when teaching ended


def test_stop_freezes_and_returns_to_the_menu():
    ws.CUBE_HSV = {"blue": [((100, 130, 50), (130, 255, 255))]}
    with tempfile.TemporaryDirectory() as root:
        arm = FakeArm()
        teacher = Teacher(arm, "desk", root=root).start()
        _wait(teacher, "menu")
        _say(teacher, "record")
        _wait(teacher, "above")
        teacher.interrupt()
        state = _wait(teacher, "stopped")
        assert state["choices"] == ["home", "stay"]
        _say(teacher, "stay")
        _wait(teacher, "menu")
        assert teacher.session.examples() == []
        teacher.quit()
        _wait(teacher, "ended")


def test_colour_ranges_hue_handles_red_wrapping():
    hue, half = wc.ranges_hue([((0, 130, 60), (5, 255, 255)), ((174, 130, 60), (179, 255, 255))])
    assert hue < 3 or hue > 177, hue
    assert 5 <= half <= 7
    assert wc.ranges_hue([((100, 1, 1), (130, 255, 255))]) == (115.0, 15.0)
    assert wc.swatch(0).startswith("#") and wc.swatch(0) != wc.swatch(60)


def test_colour_preview_names_the_cube_and_spots_clashes():
    orange = _picture((14, 220, 230))
    seen = wc.preview(orange, {"blue": [((100, 130, 50), (130, 255, 255))]})
    assert seen["seen"] and seen["name"] == "orange" and seen["clash"] is None
    seen = wc.preview(orange, {"amber": [((10, 130, 50), (20, 255, 255))]})
    assert seen["clash"] == "amber"
    seen = wc.preview(orange, {"orange": [((10, 130, 50), (20, 255, 255))]})
    assert seen["clash"] is None                             # same name: saving replaces it
    assert wc.preview(_picture((0, 10, 120)), {})["seen"] is False


def _busy_table():
    """RGB picture like the real one: wooden table, a big orange arm, thin blue
    cables at the edge, and one blue cube on the table at (230, 170)."""
    hsv = np.zeros((240, 320, 3), np.uint8)
    hsv[:, :] = (15, 45, 190)                    # pale wood: little saturation
    hsv[40:200, 60:120] = (10, 220, 220)         # the orange arm
    hsv[0:90, 5:9] = (110, 200, 120)             # blue cables, cut off by the edge
    hsv[160:182, 220:242] = (115, 200, 180)      # the blue cube
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)


def test_tapping_the_cube_measures_it_and_then_it_is_detected():
    from unoq_braccio_driver import pick_learning as pl

    rgb = _busy_table()
    assert "too big" in wc.preview(rgb, {}, (90, 100))["why"]          # tapped the arm
    assert "no strong colour" in wc.preview(rgb, {}, (280, 60))["why"]  # tapped the bare table
    auto = wc.preview(rgb, {})
    assert auto["seen"] and auto["box"] == [220, 160, 242, 182]        # never the arm or the cables
    tapped = wc.preview(rgb, {}, (230, 170))
    assert tapped["seen"] and tapped["name"] == "blue" and tapped["box"] == [220, 160, 242, 182]
    cubes = pl.detect_cubes(rgb, {"blue": tapped["ranges"]})
    assert [c["color"] for c in cubes] == ["blue"]
    assert cubes[0]["box"] == [220, 160, 242, 182]


def test_saving_colours_keeps_the_rest_of_the_setup_file():
    import yaml

    with tempfile.TemporaryDirectory() as root:
        path = os.path.join(root, "setup.yaml")
        with open(path, "w") as handle:
            handle.write("# Written by real_setup.\n#   keep me\ncamera_height_mm: 550.0\ncube_hsv:\n  blue: []\n")
        wc.save_cube_hsv(path, {"red": [[[0, 130, 60], [5, 255, 255]]]})
        text = open(path).read()
        assert text.startswith("# Written by real_setup.\n#   keep me\n")
        data = yaml.safe_load(text)
        assert data["camera_height_mm"] == 550.0
        assert data["cube_hsv"] == {"red": [[[0, 130, 60], [5, 255, 255]]]}


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
