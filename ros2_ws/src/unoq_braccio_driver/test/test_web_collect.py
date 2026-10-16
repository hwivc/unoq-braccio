"""Auto-collect: one taught example swept along the base, on a simulated table with a
camera. Needs numpy and OpenCV, no ROS: ``python test_web_collect.py`` or ``pytest``.
"""

import os
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cv2  # noqa: E402

from unoq_braccio_driver import braccio_workspace as ws  # noqa: E402
from unoq_braccio_driver import pick_learning as pl  # noqa: E402
from unoq_braccio_driver.web_collect import Collector, plan_bases  # noqa: E402

GRAB = [90, 70, 140, 150, 90, 10]     # the seed: fingers around a cube on the table
ABOVE = [90, 55, 140, 150, 90, 10]
GRAB2 = [90, 85, 125, 140, 90, 10]    # a second seed, taught further out on the same line
ABOVE2 = [90, 70, 125, 140, 90, 10]
ROWS = {tuple(GRAB[1:4]): 210, tuple(GRAB2[1:4]): 330}   # posture -> how far out it reaches (pixel row)


def spot(base, row=210):
    """Where a cube put down with a seed posture at ``base`` lands, in pixels (640 x 480)."""
    return 100 + 4 * (base - 30), row


class World:
    """A table, one cube, and an arm that grips it when the fingers close around it."""

    def __init__(self, lost_at=()):
        self.servo = [90, 90, 90, 90, 90, ws.GRIPPER_OPEN]
        self.target = list(self.servo)
        self.cube = spot(90)        # (x, y) on the table, or None while held / gone
        self.held = False
        self.lost_at = set(lost_at)  # base angles where a dropped cube rolls out of view

    def row(self, pose):
        """The table row a pose reaches when it is (within 3 degrees of) a taught posture, else None."""
        for posture, row in ROWS.items():
            if max(abs(a - b) for a, b in zip(pose[1:4], posture)) <= 3:
                return row
        return None

    def low(self, pose):
        return self.row(pose) is not None

    def send(self, pose):
        pose = [int(round(v)) for v in pose]
        closing = pose[5] >= 60 and self.target[5] < 60
        opening = pose[5] < 60 and self.target[5] >= 60
        if closing and self.low(pose) and self.cube is not None:
            x, y = spot(pose[0], self.row(pose))
            if abs(x - self.cube[0]) <= 10 and abs(y - self.cube[1]) <= 10:
                self.held, self.cube = True, None
        if opening and self.held and self.low(pose):
            self.held = False
            self.cube = None if pose[0] in self.lost_at else spot(pose[0], self.row(pose))
        self.target = pose
        self.servo = list(pose)
        return pose

    def freeze(self):
        self.send(self.servo)

    def picture(self):
        hsv = np.zeros((480, 640, 3), np.uint8)
        hsv[:, :] = (0, 10, 140)
        if self.cube is not None:
            x, y = self.cube
            hsv[y - 15:y + 15, x - 15:x + 15] = (115, 200, 200)
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)

    def latest_frame(self):
        return self.picture()

    def fresh_frames(self, count, timeout=6.0):
        return [self.picture()] * count


def _session(root, line=False):
    """A session with the near seed (and, with ``line``, a second one further out)."""
    session = pl.Session("desk", root=root)
    for name, above, grab, row in (("seed", ABOVE, GRAB, 210), ("seed2", ABOVE2, GRAB2, 330))[:2 if line else 1]:
        x, y = spot(90, row)
        session.add_example({"id": name, "kind": "teach", "u": x / 640, "v": y / 640, "angle": 0.0,
                             "area_frac": 900 / (640 * 480), "color": "blue", "width": 640, "height": 480,
                             "jitter": 0.0, "above": above, "grab": grab, "closed": 95, "lift": above[:5] + [95]})
    return session


def _wait(worker, phase, timeout=30.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        state = worker.state()
        if state["phase"] == phase and state["choices"]:
            return state
        time.sleep(0.02)
    raise AssertionError(f"never reached {phase}; at {worker.state()['phase']}: {worker.state()['prompt']}")


def _say(worker, choice):
    phase = worker.state()["phase"]
    ok, message = worker.answer(choice)
    assert ok, message
    end = time.monotonic() + 10
    while worker.state()["phase"] == phase and worker.state()["choices"] and time.monotonic() < end:
        time.sleep(0.02)            # until the worker has taken the answer


def test_plans_every_step_between_the_ends_but_not_the_seed_or_done_angles():
    assert plan_bases(90, 60, 120, 10) == [60, 70, 80, 100, 110, 120]
    assert plan_bases(90, 120, 60, 10, done=[70]) == [60, 80, 100, 110, 120]


def test_one_example_becomes_a_whole_arc():
    ws.CUBE_HSV = {"blue": [((100, 130, 50), (130, 255, 255))]}
    with tempfile.TemporaryDirectory() as root:
        _session(root)
        world = World()
        collector = Collector(world, "desk", root=root).start()
        _wait(collector, "collect_setup")
        world.send([60] + world.servo[1:])                  # the person turns the base right ...
        assert collector.update({"set_end": "right"}, 60)[0]
        world.send([120] + world.servo[1:])                 # ... and left, and picks 10 degree steps
        assert collector.update({"set_end": "left", "step": 10}, 120)[0]
        state = _wait(collector, "collect_setup")
        assert state["collect"]["planned"] == 6 and state["collect"]["window"] == [60, 120]
        _say(collector, "start")
        state = _wait(collector, "collect_setup", timeout=60)
        assert state["collect"]["progress"]["saved"] == 6, state["collect"]["progress"]
        swept = [e for e in collector.session.examples() if e["kind"] == "sweep"]
        assert sorted(e["grab"][0] for e in swept) == [60, 70, 80, 100, 110, 120]
        for e in swept:                                     # each one: where the camera saw the cube
            assert abs(e["u"] * 640 - spot(e["grab"][0])[0]) < 2
            assert e["grab"][1:4] == GRAB[1:4] and e["closed"] == 95
        assert world.cube == spot(90) and not world.held    # cube put back where the seed was taught
        assert state["collect"]["planned"] == 0             # nothing left to do on this arc
        assert len(state["spots"]) == 7
        _say(collector, "quit")


def test_a_lost_cube_is_asked_for_and_the_spot_skipped():
    ws.CUBE_HSV = {"blue": [((100, 130, 50), (130, 255, 255))]}
    with tempfile.TemporaryDirectory() as root:
        _session(root)
        world = World(lost_at={100})
        collector = Collector(world, "desk", root=root).start()
        _wait(collector, "collect_setup")
        collector.update({"set_end": "right", "step": 10}, 80)
        collector.update({"set_end": "left"}, 110)
        _say(collector, "start")
        _wait(collector, "collect_lost", timeout=60)        # fell out of view at 100 degrees
        world.cube = spot(90)                               # the person puts it back
        _say(collector, "ok")
        state = _wait(collector, "collect_setup", timeout=60)
        progress = state["collect"]["progress"]
        assert (progress["saved"], progress["skipped"]) == (2, 1), progress
        assert sorted(e["grab"][0] for e in collector.session.examples() if e["kind"] == "sweep") == [80, 110]
        _say(collector, "quit")


def test_sweep_all_turns_a_taught_line_into_a_grid():
    ws.CUBE_HSV = {"blue": [((100, 130, 50), (130, 255, 255))]}
    with tempfile.TemporaryDirectory() as root:
        _session(root, line=True)                           # two examples: near, then further out
        world = World()
        collector = Collector(world, "desk", root=root).start()
        _wait(collector, "collect_setup")
        collector.update({"set_end": "right", "step": 10, "mode": "all"}, 70)
        collector.update({"set_end": "left"}, 110)
        state = _wait(collector, "collect_setup")
        assert state["collect"]["planned"] == 8             # 4 new spots on each of 2 arcs
        _say(collector, "start")
        state = _wait(collector, "collect_setup", timeout=90)
        progress = state["collect"]["progress"]
        assert (progress["arcs"], progress["saved"], progress["skipped"]) == (2, 8, 0), progress
        swept = [e for e in collector.session.examples() if e["kind"] == "sweep"]
        rows = sorted({round(e["v"] * 640) for e in swept})
        assert rows == [210, 330]                           # one arc per taught example
        assert [e["seed"] for e in swept] == ["seed"] * 4 + ["seed2"] * 4   # near arc first
        assert state["collect"]["planned"] == 0
        _say(collector, "quit")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
