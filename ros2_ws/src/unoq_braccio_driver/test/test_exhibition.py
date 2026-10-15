"""Exhibition mode's rules: where cubes may be put down and which to pick.
Needs PyYAML only (numpy for the model test), no ROS: ``python test_exhibition.py`` or ``pytest``.
"""

import os
import random
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from unoq_braccio_driver import exhibition as ex  # noqa: E402


def _example(x, y, base):
    """A taught example at (x, y) px (640 wide) whose grab turned the base to ``base``."""
    return {"u": x / 640.0, "v": y / 640.0, "angle": 0.0, "closed": 95,
            "above": [base, 80, 120, 140, 90, 10], "grab": [base, 70, 140, 150, 90, 10]}


def _cube(x, y, color="blue"):
    return {"u": x / 640.0, "v": y / 640.0, "angle": 0.0, "color": color,
            "area_frac": (30 * 30) / (640 * 480), "width": 640, "height": 480}


EXAMPLES = [_example(100 + 60 * i, 200, base) for i, base in enumerate((40, 60, 80, 100, 120, 140))]


def test_only_taught_spots_inside_the_base_window_are_used():
    everything = ex.place_spots(EXAMPLES, ex.clean({}))
    assert len(everything) == 6
    window = ex.clean({"base_min": 60, "base_max": 120})
    spots = ex.place_spots(EXAMPLES, window)
    assert [s["base"] for s in spots] == [60, 80, 100, 120]
    twice = EXAMPLES + [_example(101, 201, 40)]          # taught twice at almost one place
    assert len(ex.place_spots(twice, ex.clean({}))) == 6


def test_never_puts_a_cube_on_or_next_to_another():
    spots = ex.place_spots(EXAMPLES, ex.clean({}))
    cubes = [_cube(100, 200), _cube(160, 200), _cube(400, 200)]
    picked = cubes[0]
    spacing = 2 * ex.cube_size_px(cubes)
    assert 55 < spacing < 65                               # two 30 px cubes
    for seed in range(30):
        spot = ex.choose_place(spots, cubes, picked, spacing, rng=random.Random(seed))
        for c in cubes:                                    # including where the picked cube was
            assert abs(spot["x"] - c["u"] * 640) + abs(spot["y"] - c["v"] * 640) >= spacing
    crowded = [_cube(100 + 60 * i, 200) for i in range(6)]
    assert ex.choose_place(spots, crowded, crowded[0], spacing) is None


def test_varies_where_it_puts_cubes():
    spots = ex.place_spots(EXAMPLES, ex.clean({}))
    cubes = [_cube(100, 200)]
    chosen = {ex.choose_place(spots, cubes, cubes[0], 60, rng=random.Random(seed))["base"] for seed in range(40)}
    assert len(chosen) >= 3
    recent = [(s["u"], s["v"]) for s in spots if s["base"] != 140]
    only = ex.choose_place(spots, cubes, cubes[0], 60, recent=recent)
    assert only["base"] == 140                             # avoids spots it just used


def test_only_picks_known_cubes_inside_the_window():
    window = ex.clean({"base_min": 60, "base_max": 120})

    def predict(cube):
        base = 40 + (cube["u"] * 640 - 100) / 60 * 20
        return {"nearest_px": 0 if cube["color"] != "far" else 200,
                "above": [base, 80, 120, 140, 90, 10], "grab": [base, 70, 140, 150, 90, 10]}

    cubes = [_cube(100, 200), _cube(220, 200), _cube(280, 200, "far")]
    cube, guess = ex.choose_pick(cubes, predict, window, 60, {}, rng=random.Random(1))
    assert cube is cubes[1]                                # base 80: inside; 40 is outside, "far" unknown
    failures = {ex.spot_key(cubes[1]): 2}
    assert ex.choose_pick(cubes, predict, window, 60, failures)[0] is None
    assert "No cubes" in ex.choose_pick([], predict, window, 60, {})[1]


def test_settings_are_cleaned_and_kept():
    with tempfile.TemporaryDirectory() as root:
        path = os.path.join(root, "exhibition.yaml")
        assert ex.load_settings(path) == ex.DEFAULTS
        saved = ex.save_settings({"base_min": 150, "base_max": 30, "rest_s": 1, "moves_per_round": 99}, path)
        assert (saved["base_min"], saved["base_max"]) == (30, 150)   # swapped into order
        assert saved["rest_s"] == 5 and saved["moves_per_round"] == 20
        assert ex.load_settings(path) == saved
    assert ex.in_window(90, ex.clean({})) and not ex.in_window(10, ex.clean({"base_min": 20}))


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
