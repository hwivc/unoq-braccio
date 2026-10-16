"""Exhibition mode: where to pick and where to put cubes down so the table never empties. No ROS.

The arm shuffles cubes around the table for hours: pick one, place it on a
free spot, repeat, resting in between. Placing needs no new teaching: a
taught example is a spot where a cube really sat on the table and the arm
really picked it up, so putting a cube down there is the same motion with
the gripper opening instead of closing. Places are therefore only ever
taught spots (never past the table edge), only inside the base window set
on the web page (``base_min`` .. ``base_max`` servo degrees), and at least
``spacing`` away from every other cube.

Rounds follow a playlist of patterns the audience can recognise (line up by
colour, a circle, colour zones, a swap, a shuffle). A pattern is only ever a
choice of known spots (taught or auto-collected examples), so placing needs
no new maths: the arm puts a cube down with that example's own pose.
``pattern_targets`` chooses the spots; ``next_move`` decides, from what the
camera sees now, which cube goes where next (clearing a blocking cube to a
free spot first when needed).

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
    "patterns": ["line", "arc", "zones", "swap", "shuffle"],   # the playlist, in this order
}
PATTERNS = {
    "line": "Lining up by colour",
    "arc": "Making a circle",
    "zones": "Sorting into colour zones",
    "swap": "Swapping two cubes",
    "shuffle": "Shuffling",
}
TARGET_TOLERANCE_PX = 18.0   # a cube this close to a target counts as on it
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
    patterns = [p for p in (out["patterns"] or []) if p in PATTERNS]
    out["patterns"] = [p for p in PATTERNS if p in patterns] or ["shuffle"]
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
    """Known spots a cube may be put down on, inside the base window:
    [{u, v, x, y, base, above, grab, group}].

    Each is an example (taught or auto-collected): where a cube really sat
    and the pose that really picked it up, so putting a cube down there uses
    that same pose. ``group`` is the taught example an auto-collected arc came
    from (an arc's spots lie on one circle). Examples at almost the same place
    are merged.
    """
    spots = []
    for i, e in enumerate(examples):
        base = e["grab"][0]
        if not in_window(base, settings):
            continue
        x, y = px(e["u"], e["v"])
        if any(math.hypot(x - s["x"], y - s["y"]) < SPOT_MERGE_PX for s in spots):
            continue
        spots.append({"u": e["u"], "v": e["v"], "x": x, "y": y, "base": base,
                      "above": list(e.get("above") or []), "grab": list(e["grab"]),
                      "group": e.get("seed") or e.get("id") or f"example{i}"})
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


# -- patterns: which known spots, and which cube moves next -----------------------------

def spaced(spots, spacing):
    """Spots in the given order, keeping only those ``spacing`` from every one kept before."""
    kept = []
    for spot in spots:
        if all(math.hypot(spot["x"] - k["x"], spot["y"] - k["y"]) >= spacing for k in kept):
            kept.append(spot)
    return kept


def evenly(items, count):
    """``count`` items spread evenly over ``items`` (first and last included)."""
    if count >= len(items):
        return list(items)
    if count == 1:
        return [items[len(items) // 2]]
    return [items[round(i * (len(items) - 1) / (count - 1))] for i in range(count)]


def snap(spots, u, v, within=25.0):
    """The known spot nearest (u, v), if within ``within`` pixels."""
    x, y = px(u, v)
    best = min(spots, key=lambda s: math.hypot(s["x"] - x, s["y"] - y), default=None)
    return best if best is not None and math.hypot(best["x"] - x, best["y"] - y) <= within else None


def _target(spot, color):
    return {"u": spot["u"], "v": spot["v"], "x": spot["x"], "y": spot["y"], "color": color, "spot": spot}


def _freshest(options, cubes, rng):
    """Of several possible layouts, one that needs the most cubes moved (so the audience
    sees something happen), at random among equals."""
    scored = [(len(satisfied(targets, cubes)), targets) for targets in options]
    least = min(score for score, _ in scored)
    return rng.choice([targets for score, targets in scored if score == least])


def pattern_targets(name, spots, cubes, spacing, rng=random):
    """Targets for a pattern: [{u, v, x, y, color, spot}] (color None = any cube), or None
    when these spots cannot make it with these cubes."""
    n = len(cubes)
    colors = sorted(c["color"] for c in cubes)
    if n == 0:
        return None
    if name == "line":       # a row across the picture, cubes left to right by colour
        rows, row = [], []
        for spot in sorted(spots, key=lambda s: s["y"]):
            if row and spot["y"] - row[0]["y"] > 25:
                rows.append(row)
                row = []
            row.append(spot)
        rows.append(row)
        usable = [spaced(sorted(r, key=lambda s: s["x"]), spacing) for r in rows]
        usable = [r for r in usable if len(r) >= n]
        if not usable:
            return None
        return _freshest([[_target(s, c) for s, c in zip(evenly(r, n), colors)] for r in usable], cubes, rng)
    if name == "arc":        # the spots of one collected arc: they lie on a circle
        groups = {}
        for spot in spots:
            groups.setdefault(spot["group"], []).append(spot)
        usable = [spaced(sorted(g, key=lambda s: s["base"]), spacing) for g in groups.values()]
        usable = [g for g in usable if len(g) >= max(n, 3)]
        if not usable:
            return None
        return _freshest([[_target(s, c) for s, c in zip(evenly(g, n), colors)] for g in usable], cubes, rng)
    if name == "zones":      # each colour gets its own side of the table
        kinds = sorted(set(colors))
        if len(kinds) < 2:
            return None
        xs = sorted(s["x"] for s in spots)
        targets, kept = [], []
        for i, color in enumerate(kinds):
            lo, hi = xs[len(xs) * i // len(kinds)], xs[min(len(xs) - 1, len(xs) * (i + 1) // len(kinds) - 1)]
            band = [s for s in spots if lo <= s["x"] <= hi]
            free = spaced(kept + rng.sample(band, len(band)), spacing)[len(kept):]
            want = colors.count(color)
            if len(free) < want:
                return None
            kept += free[:want]
            targets += [_target(s, color) for s in free[:want]]
        return targets
    if name == "swap":       # two cubes of different colours trade places, the rest stay
        placed = [(c, snap(spots, c["u"], c["v"])) for c in cubes]
        if any(spot is None for _, spot in placed) or len(set(colors)) < 2:
            return None
        a, b = rng.sample(placed, 2)
        while a[0]["color"] == b[0]["color"]:
            a, b = rng.sample(placed, 2)
        targets = [_target(spot, c["color"]) for c, spot in placed if c is not a[0] and c is not b[0]]
        return targets + [_target(b[1], a[0]["color"]), _target(a[1], b[0]["color"])]
    if name == "shuffle":    # everyone to a new free spot
        taken = [px(c["u"], c["v"]) for c in cubes]
        fresh = [s for s in spots if all(math.hypot(s["x"] - x, s["y"] - y) >= spacing for x, y in taken)]
        chosen = spaced(rng.sample(fresh, len(fresh)), spacing)
        if len(chosen) < n:
            return None
        shuffled = list(colors)
        rng.shuffle(shuffled)
        return [_target(s, c) for s, c in zip(chosen, shuffled)]
    raise ValueError(name)


def satisfied(targets, cubes, tolerance=TARGET_TOLERANCE_PX):
    """{target index: cube} for targets with a matching cube on them (each cube counts once)."""
    done, used = {}, set()
    for i, t in enumerate(targets):
        for j, c in enumerate(cubes):
            if j in used or (t["color"] is not None and c["color"] != t["color"]):
                continue
            if math.hypot(c["u"] * 640 - t["x"], c["v"] * 640 - t["y"]) <= tolerance:
                done[i], used = c, used | {j}
                break
    return done


def next_move(targets, cubes, spots, spacing, can_pick=lambda cube: True):
    """What to do next for a pattern, from the cubes seen now:
    ("done", None, None), ("move", cube, spot) or ("stuck", why, None).

    A cube already on a matching target stays. A target blocked by another
    cube first gets that cube moved to a free spot that is not a target.
    """
    done = satisfied(targets, cubes)
    if len(done) == len(targets):
        return "done", None, None
    resting = {id(c) for c in done.values()}
    loose = [c for c in cubes if id(c) not in resting and can_pick(c)]
    for i, t in enumerate(targets):
        if i in done:
            continue
        blocker = next((c for c in cubes if id(c) not in resting
                        and math.hypot(c["u"] * 640 - t["x"], c["v"] * 640 - t["y"]) < spacing), None)
        if blocker is not None:
            if not can_pick(blocker):
                continue
            if t["color"] is None or blocker["color"] == t["color"]:
                return "move", blocker, t["spot"]          # right cube, just off the mark
            others = [c for c in cubes if c is not blocker]
            free = [s for s in free_spots(spots, others, spacing, avoid=[(blocker["u"], blocker["v"])])
                    if all(math.hypot(s["x"] - u["x"], s["y"] - u["y"]) >= spacing for u in targets)]
            if not free:
                continue
            return "move", blocker, min(free, key=lambda s: math.hypot(
                s["x"] - blocker["u"] * 640, s["y"] - blocker["v"] * 640))
        fits = [c for c in loose if t["color"] is None or c["color"] == t["color"]]
        if fits:
            cube = min(fits, key=lambda c: math.hypot(c["u"] * 640 - t["x"], c["v"] * 640 - t["y"]))
            return "move", cube, t["spot"]
    return "stuck", "No cube it can move to finish the pattern.", None
