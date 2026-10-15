"""Exhibition mode: where to pick and where to put cubes down so the table never empties. No ROS.

The arm shuffles cubes around the table for hours: pick one, place it on a
free spot, repeat, resting in between. Placing needs no new teaching: a
taught example is a spot where a cube really sat on the table and the arm
really picked it up, so putting a cube down there is the same motion with
the gripper opening instead of closing. Places are therefore only ever
taught spots (never past the table edge), only inside the base window set
on the web page (``base_min`` .. ``base_max`` servo degrees), and at least
``spacing`` away from every other cube.

Settings live in ~/.ros/braccio_exhibition.yaml (the web page's Show tab
writes it; exhibition_demo re-reads it every round).
"""

from __future__ import annotations

import math
import os
import random

SETTINGS_FILE = "~/.ros/braccio_exhibition.yaml"
DEFAULTS = {
    "base_min": None,        # servo degrees; None = no limit on that side
    "base_max": None,
    "rest_s": 30,            # rest at the upright pose between rounds (servos cool down)
    "moves_per_round": 3,
    "attract": True,         # a wave / nod / dance every few rests
    "attract_every": 2,      # rounds between fun moves
    "max_misses": 3,         # misses in a row before asking for help
    "max_minutes": 0,        # 0 = run until stopped
}
ATTRACT_TRICKS = ("wave", "nod", "dance", "shake")   # all upright: never touch the table
SPOT_MERGE_PX = 12.0       # taught examples closer than this count as one spot


def load_settings(path=SETTINGS_FILE):
    import yaml

    settings = dict(DEFAULTS)
    try:
        with open(os.path.expanduser(path), encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except (OSError, ValueError):
        return settings
    for key, default in DEFAULTS.items():
        if key in data:
            settings[key] = data[key]
    return clean(settings)


def clean(settings):
    """Settings with sane types and ranges (from a file or a web request)."""
    out = dict(DEFAULTS)
    out.update({k: v for k, v in settings.items() if k in DEFAULTS})
    for key in ("base_min", "base_max"):
        out[key] = None if out[key] is None else max(0, min(180, int(out[key])))
    if out["base_min"] is not None and out["base_max"] is not None and out["base_min"] > out["base_max"]:
        out["base_min"], out["base_max"] = out["base_max"], out["base_min"]
    out["rest_s"] = max(5, min(600, int(out["rest_s"])))
    out["moves_per_round"] = max(1, min(20, int(out["moves_per_round"])))
    out["attract"] = bool(out["attract"])
    out["attract_every"] = max(1, min(20, int(out["attract_every"])))
    out["max_misses"] = max(1, min(10, int(out["max_misses"])))
    out["max_minutes"] = max(0, min(24 * 60, int(out["max_minutes"])))
    return out


def save_settings(settings, path=SETTINGS_FILE):
    import yaml

    settings = clean(settings)
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write("# Exhibition mode, set from the web page's Show tab.\n")
        yaml.safe_dump(settings, handle, sort_keys=False)
    os.replace(tmp, path)
    return settings


def in_window(base, settings):
    lo, hi = settings.get("base_min"), settings.get("base_max")
    return (lo is None or base >= lo) and (hi is None or base <= hi)


def px(u, v):
    """A (u, v) reading as pixels at 640 wide (how pick_learning measures distance)."""
    return u * 640.0, v * 640.0


def place_spots(examples, settings):
    """Taught spots a cube may be put down on: [{u, v, base}], inside the base window.

    Uses each example's grab pose (where the arm really reached the table)
    and merges examples taught at almost the same place.
    """
    spots = []
    for e in examples:
        base = e["grab"][0]
        if not in_window(base, settings):
            continue
        x, y = px(e["u"], e["v"])
        if any(math.hypot(x - s["x"], y - s["y"]) < SPOT_MERGE_PX for s in spots):
            continue
        spots.append({"u": e["u"], "v": e["v"], "x": x, "y": y, "base": base})
    return spots


def cube_size_px(cubes, default=30.0):
    """Typical cube side in pixels at 640 wide, from what the camera sees."""
    sides = []
    for c in cubes:
        if c.get("area_frac") and c.get("width") and c.get("height"):
            side = math.sqrt(c["area_frac"] * c["width"] * c["height"]) * 640.0 / c["width"]
            sides.append(side)
    if not sides:
        return default
    sides.sort()
    return sides[len(sides) // 2]


def free_spots(spots, cubes, spacing_px, avoid=()):
    """Spots at least ``spacing_px`` from every cube on the table (and from ``avoid`` points)."""
    taken = [px(c["u"], c["v"]) for c in cubes] + [px(u, v) for u, v in avoid]
    return [s for s in spots if all(math.hypot(s["x"] - x, s["y"] - y) >= spacing_px for x, y in taken)]


def choose_place(spots, cubes, picked, spacing_px, recent=(), rng=random):
    """Where to put the picked cube down, or None when no spot is free.

    Prefers spots far from the cube's old place and not used recently, with
    some chance in it so the show does not look like a loop.
    """
    others = [c for c in cubes if c is not picked]
    free = free_spots(spots, others, spacing_px, avoid=[(picked["u"], picked["v"])])
    if not free:
        return None
    fresh = [s for s in free if all(math.hypot(s["x"] - x, s["y"] - y) >= SPOT_MERGE_PX
                                    for x, y in (px(u, v) for u, v in recent))] or free
    ox, oy = px(picked["u"], picked["v"])
    weights = [1.0 + math.hypot(s["x"] - ox, s["y"] - oy) / 100.0 for s in fresh]
    return rng.choices(fresh, weights=weights, k=1)[0]


def choose_pick(cubes, predict, settings, unsure_px, failures, max_tries=2, rng=random):
    """(cube, guess) to pick next, or (None, why).

    Only cubes the model knows (within ``unsure_px`` of an example), whose
    grab turns the base inside the window, and that have not been missed
    ``max_tries`` times where they lie.
    """
    options = []
    for cube in cubes:
        guess = predict(cube)
        if guess["nearest_px"] > unsure_px:
            continue
        if not in_window(guess["grab"][0], settings) or not in_window(guess["above"][0], settings):
            continue
        if failures.get(spot_key(cube), 0) >= max_tries:
            continue
        options.append((cube, guess))
    if not options:
        if not cubes:
            return None, "No cubes on the table."
        return None, "No cube it can reach inside the base window (or only ones it missed)."
    return rng.choice(options)


def spot_key(cube):
    """Cubes are told apart by where they lie (to count misses per place)."""
    x, y = px(cube["u"], cube["v"])
    return (round(x / 15.0), round(y / 15.0))
