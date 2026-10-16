"""exhibition_demo's show loop against a simulated table: a fake arm that really picks up
and puts down cubes where the learned poses say. Needs ROS 2 Python (rclpy) to import the
node, but no robot, camera or running ROS graph: ``python test_exhibition_demo.py``.
"""

import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    from unoq_braccio_driver import exhibition_demo as demo_module
except ImportError:  # no ROS here: nothing to test
    demo_module = None

from unoq_braccio_driver import exhibition as ex  # noqa: E402
from unoq_braccio_driver import pick_learning as pl  # noqa: E402

OPEN, CLOSED = 10, 95


def _pose(u, v):
    """The made-up arm: grab pose for a cube at (u, v)."""
    base = 30 + 120 * u
    return [round(base), round(60 + 40 * v), round(150 - 30 * v), 150, 90]


def _examples():
    out = []
    for u in np.linspace(0.25, 0.75, 6):
        for v in np.linspace(0.2, 0.45, 4):
            grab = _pose(u, v)
            above = [grab[0], grab[1] - 15] + grab[2:]
            out.append({"u": float(u), "v": float(v), "angle": 0.0, "closed": CLOSED,
                        "above": above + [OPEN], "grab": grab + [OPEN]})
    return out


class Stop(Exception):
    pass


class World:
    """Cubes on a table; the fake arm grips the cube under its grab pose and drops it where it opens."""

    def __init__(self, model, cubes):
        self.model, self.cubes, self.held = model, list(cubes), None
        self.low_poses = []          # every pose sent at table height (grab level)

    def reading(self, cube):
        return {"u": cube["u"], "v": cube["v"], "angle": 0.0, "color": cube["color"],
                "area_frac": 900 / (640 * 480), "width": 640, "height": 480, "jitter": 0.0}

    def at(self, pose, u, v):
        grab = self.model.predict({"u": u, "v": v, "angle": 0.0})["grab"]
        return max(abs(a - b) for a, b in zip(grab[:4], pose[:4])) <= 2

    def move(self, pose):
        if self.held is None and pose[5] == CLOSED:
            for cube in self.cubes:
                if self.at(pose, cube["u"], cube["v"]):
                    self.low_poses.append(list(pose))
                    self.held = cube
                    self.cubes.remove(cube)
                    return
        if self.held is not None and pose[5] == OPEN:
            for e in _examples():
                if self.at(pose, e["u"], e["v"]):
                    self.low_poses.append(list(pose))
                    self.cubes.append({**self.held, "u": e["u"], "v": e["v"]})
                    self.held = None
                    return


class Param:
    def __init__(self, value):
        self.value = value


class Publisher:
    def __init__(self, sink):
        self.sink = sink

    def publish(self, msg):
        self.sink.append(msg.data)


class Logger:
    def info(self, *_a, **_k):
        pass

    warning = error = info


def _demo(world, settings, rounds, on_round=None):
    """An ExhibitionDemo wired to ``world`` instead of ROS, stopping after ``rounds`` rests."""
    demo = object.__new__(demo_module.ExhibitionDemo)
    states, currents = [], []
    demo.state_pub, demo.current_pub = Publisher(states), Publisher(currents)
    demo.model, demo.examples = world.model, _examples()
    demo.current = {"mode": "exhibition", "moved": 0, "picks": 0, "misses": 0, "round": 0,
                    "started": 0.0, "attention": ""}
    demo.rng = __import__("random").Random(3)
    demo.home = [90, 90, 90, 90, 90, OPEN]
    demo.gripper_open, demo.gripper_closed = OPEN, CLOSED
    demo.target = list(demo.home)
    demo.session = type("S", (), {"name": "sim", "reference": lambda self: None})()
    params = {"unsure_px": 60.0, "frames": 3}
    demo.get_parameter = lambda name: Param(params[name])
    demo.get_logger = lambda: Logger()
    demo.settings = lambda: ex.clean(settings)
    demo.wait_ready = lambda: True

    def move(pose, stop=None, settle=0.2):
        demo.target = [int(round(v)) for v in pose]
        world.move(demo.target)
    demo.move = move
    demo.send = lambda pose: move(pose)
    demo.read_cubes = lambda: [world.reading(c) for c in world.cubes]
    demo.fresh_frames = lambda n: ["frame"] * n
    demo.detect = lambda frame: [world.reading(c) for c in world.cubes]
    demo.attract = lambda: states.append("ATTRACT")
    count = {"rests": 0}

    def rest(seconds, label=""):
        count["rests"] += 1
        if on_round:
            on_round(count["rests"])
        if count["rests"] >= rounds:
            raise Stop()
    demo.rest = rest
    demo_module.time.sleep = lambda s: None
    return demo, states, currents


def _model():
    return pl.PickModel.fit(_examples(), pl.Knn(4))


def test_shuffles_for_rounds_inside_the_window_without_losing_cubes():
    if demo_module is None:
        print("skip: no rclpy")
        return
    start = [{"u": 0.25, "v": 0.2, "color": "blue"}, {"u": 0.55, "v": 0.37, "color": "red"},
             {"u": 0.75, "v": 0.45, "color": "yellow"}]
    world = World(_model(), start)
    settings = {"base_min": 60, "base_max": 150, "moves_per_round": 3, "attract_every": 1, "patterns": ["shuffle"]}
    demo, states, _ = _demo(world, settings, rounds=3)
    try:
        demo.run()
    except Stop:
        pass
    assert demo.current["moved"] == 9, (demo.current, states[-12:])
    assert demo.current["misses"] == 0 and world.held is None
    assert sorted(c["color"] for c in world.cubes) == ["blue", "red", "yellow"]
    for pose in world.low_poses:                       # every touch of the table inside the window
        assert 60 <= pose[0] <= 150, pose
    for i, a in enumerate(world.cubes):                # never stacked or touching
        for b in world.cubes[i + 1:]:
            assert math.hypot(a["u"] - b["u"], a["v"] - b["v"]) * 640 >= 50
    assert "ATTRACT" in states and "ATTENTION" not in states
    print("  moved", demo.current["moved"], "cubes in", states.count("PLACED"), "places")


def test_asks_for_help_when_a_cube_disappears():
    if demo_module is None:
        print("skip: no rclpy")
        return
    start = [{"u": 0.25, "v": 0.2, "color": "blue"}, {"u": 0.75, "v": 0.45, "color": "red"}]
    world = World(_model(), start)

    def lose_one(rests):
        if rests == 1 and world.cubes:
            world.cubes.pop()                              # a visitor took it, or it fell off
    demo, states, currents = _demo(world, {"moves_per_round": 1, "attract": False, "patterns": ["shuffle"]},
                                   rounds=3, on_round=lose_one)
    calls = {"n": 0}

    def attention(why, settings):                      # stop after the first call for help
        states.append("ATTENTION")
        calls["n"] += 1
        demo.current["attention"] = why
        raise Stop()
    demo.attention = attention
    try:
        demo.run()
    except Stop:
        pass
    assert calls["n"] == 1 and "missing" in demo.current["attention"], demo.current


def test_plays_a_line_pattern_on_known_spots():
    if demo_module is None:
        print("skip: no rclpy")
        return
    start = [{"u": 0.35, "v": 0.2, "color": "red"}, {"u": 0.55, "v": 0.45, "color": "blue"},
             {"u": 0.75, "v": 0.3667, "color": "yellow"}]
    world = World(_model(), start)
    demo, states, currents = _demo(world, {"patterns": ["line"], "attract_every": 1}, rounds=1)
    demo.unsure = 60.0
    try:
        demo.run()
    except Stop:
        pass
    assert "PATTERN_DONE" in states, states[-10:]
    rows = {round(c["v"], 3) for c in world.cubes}
    assert len(rows) == 1, world.cubes                      # all on one row ...
    order = [c["color"] for c in sorted(world.cubes, key=lambda c: c["u"])]
    assert order == sorted(order)                           # ... left to right by colour
    last = json.loads(currents[-1])
    assert last["narration"].endswith("done!") and all(t[3] for t in last["targets"])


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
