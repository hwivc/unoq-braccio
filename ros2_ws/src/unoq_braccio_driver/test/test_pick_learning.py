"""pick_learning: camera reading -> servo angles. Needs numpy and OpenCV, no ROS:
``python test_pick_learning.py`` or ``pytest``.
"""

import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from unoq_braccio_driver import pick_learning as pl  # noqa: E402


def _fake_arm(u, v, angle):
    """A made-up arm with a 40 degree shoulder offset: the model must not care."""
    base = 60 + 80 * u + 10 * v
    shoulder = 70 + 30 * v + 5 * u * u + 40
    roll = 90 - angle + (base - 90)
    roll -= 90 * round((roll - 90) / 90)
    grab = [base, shoulder, 160 - 20 * v, 150 + 10 * u * v, roll, 10]
    above = [base, shoulder - 20] + grab[2:]
    return above, grab


def _examples(n, seed=1, noise=0.5):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        u, v, a = rng.uniform(0.2, 0.8), rng.uniform(0.1, 0.5), rng.uniform(-45, 45)
        above, grab = _fake_arm(u, v, a)
        out.append({
            "u": u, "v": v, "angle": a, "closed": 95,
            "above": [round(x + rng.normal(0, noise)) for x in above],
            "grab": [round(x + rng.normal(0, noise)) for x in grab],
        })
    return out


def test_learns_unseen_cubes_within_two_degrees():
    model, scores = pl.train(_examples(20))
    assert model.info["chosen"] in scores
    errors, roll_errors = [], []
    for e in _examples(30, seed=2, noise=0):
        guess = model.predict(e)
        errors += [abs(guess["grab"][j] - e["grab"][j]) for j in range(4)]
        errors += [abs(guess["above"][j] - e["above"][j]) for j in range(4)]
        roll_errors.append(abs(pl.wrap90(guess["grab"][4] - e["grab"][4])))
    assert np.mean(errors) < 2.0, np.mean(errors)
    assert np.mean(roll_errors) < 3.0, np.mean(roll_errors)


def test_poses_are_valid_servo_values():
    model, _ = pl.train(_examples(8))
    guess = model.predict({"u": 0.5, "v": 0.3, "angle": 44.0})
    for name in ("above", "grab", "grip", "lift"):
        assert len(guess[name]) == 6 and all(isinstance(v, int) for v in guess[name])
        assert 0 <= guess[name][4] <= 180
    assert guess["grab"][5] == 10 and guess["grip"][5] == 95 and guess["lift"][5] == 95


def test_far_from_examples_is_flagged():
    examples = _examples(10)
    model, _ = pl.train(examples)
    e = examples[3]
    near = model.predict({"u": e["u"] + 0.02, "v": e["v"], "angle": 0.0})["nearest_px"]
    far = model.predict({"u": 0.98, "v": 0.02, "angle": 0.0})["nearest_px"]
    assert near < 60 < far


def test_one_example_is_enough_to_start():
    model, _ = pl.train(_examples(1))
    e = _examples(1)[0]
    assert model.predict(e)["grab"][:4] == e["grab"][:4]


def test_session_saves_and_reloads_models():
    with tempfile.TemporaryDirectory() as root:
        session = pl.Session("desk", root=root)
        for e in _examples(6):
            session.add_example(e)
        assert len(session.examples()) == 6
        model, _ = pl.train(session.examples())
        assert session.save_model(model) == "model_0006"
        session.remove_last_example()
        assert len(session.examples()) == 5
        reading = {"u": 0.4, "v": 0.3, "angle": 10.0}
        for name in ("", "model_0006", "0006"):
            assert session.load_model(name).predict(reading) == model.predict(reading)
        assert session.models() == ["model_0006"]
        assert session.load_model("model_9999") is None


def test_review_finds_odd_posture_and_open_misses():
    examples = _examples(20)
    for i, e in enumerate(examples):
        e["id"], e["time"] = f"e{i}", f"2026-01-01 00:00:{i:02d}"
    examples[7]["grab"][1] += 40  # a very different shoulder for the same spot
    suspects = pl.suspect_examples(examples)
    assert suspects and suspects[0]["index"] == 7 and suspects[0]["joint"] == "shoulder"
    assert all(s["index"] == 7 for s in suspects)

    def test(u, v, result, second):
        return {"time": f"2026-01-01 00:01:{second:02d}", "result": result,
                "reading": {"u": u, "v": v, "angle": 0.0}, "guess": {"nearest_px": 5.0}}
    tests = [test(0.3, 0.2, "fail", 0), test(0.3, 0.2, "ok", 1), test(0.7, 0.4, "stopped", 2)]
    missed = pl.misses(tests, examples)
    assert [m["fixed"] for m in missed] == [True, False]

    with tempfile.TemporaryDirectory() as root:
        session = pl.Session("desk", root=root)
        for e in examples:
            session.add_example(e)
        assert session.remove_example("e7")["id"] == "e7"
        assert session.remove_example("e7") is None
        assert len(session.examples()) == 19
        assert pl.suspect_examples(session.examples()) == []


def _picture(cubes, shape=(360, 640)):
    """RGB picture: grey table with yellow squares at (x, y, size, angle)."""
    import cv2

    rgb = np.full(shape + (3,), 150, np.uint8)
    for x, y, size, angle in cubes:
        box = cv2.boxPoints(((x, y), (size, size), angle)).astype(np.int32)
        cv2.fillPoly(rgb, [box], (230, 210, 30))
    return rgb


YELLOW = {"yellow": [((22, 100, 100), (38, 255, 255))]}


def test_detects_position_and_angle():
    cubes = pl.detect_cubes(_picture([(320, 180, 50, 20)]), YELLOW)
    assert len(cubes) == 1
    c = cubes[0]
    assert abs(c["u"] * 640 - 320) < 2 and abs(c["v"] * 640 - 180) < 2
    assert abs(pl.wrap90(c["angle"] - 20)) < 3 or abs(pl.wrap90(c["angle"] + 20)) < 3
    assert c["color"] == "yellow"


def test_ignores_cut_off_and_tiny_blobs():
    assert pl.detect_cubes(_picture([(5, 180, 50, 0), (300, 100, 4, 0)]), YELLOW) == []


def test_summarise_needs_one_steady_cube():
    one = pl.detect_cubes(_picture([(320, 180, 50, 0)]), YELLOW)
    two = pl.detect_cubes(_picture([(200, 180, 50, 0), (400, 180, 50, 0)]), YELLOW)
    reading, why = pl.summarise([one] * 9 + [two])
    assert reading is not None and why is None
    reading, why = pl.summarise([two] * 5)
    assert reading is None and "ONE" in why
    assert len(pl.steady_cubes([two] * 5)) == 2


def test_camera_check_sees_a_shift():
    import cv2

    rng = np.random.default_rng(0)
    scene = cv2.GaussianBlur(rng.integers(0, 255, (360, 640, 3), dtype=np.uint8), (5, 5), 0)
    same = pl.compare_to_reference(scene, scene)
    assert same is not None and not pl.camera_moved(same)
    shifted = np.roll(scene, 20, axis=1)
    check = pl.compare_to_reference(shifted, scene)
    assert check is not None and pl.camera_moved(check)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
