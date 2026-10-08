"""Learn picking from demonstrations: camera reading -> servo angles.

Pure Python + numpy/OpenCV, no ROS, so it can be tested anywhere.

You show the arm how to pick a cube (``teach_pick``). Each example stores
what the overhead camera saw before the arm moved (cube centre ``u, v`` and
its angle) and the servo angles you drove the arm to: an "above" pose and a
"grab" pose. A small model then maps a new camera reading straight to servo
angles. The arm's kinematic model is never used, so servo offsets, bent
links or a camera at an odd angle do not matter: whatever the real arm did
when it picked a cube is what gets learned.

Pixel positions are stored as fractions of the image width (``u = x / W``,
``v = y / W``), so changing only the camera resolution keeps the examples
valid. Moving or zooming the camera does not: start a new session.

Colour is never a model input, only the cube's position and angle, so new
cube colours need no retraining (just their colour range and a drop pose).
"""

from __future__ import annotations

import json
import math
import os
import time

import numpy as np

ARM = 4                    # base, shoulder, elbow, wrist_vertical are learned per pose
ROLL = 4                   # wrist_rotation index in a 6-servo pose
GRIP = 5                   # gripper index
TEACH_ROOT = "~/.ros/braccio_teach"


# -- camera reading -------------------------------------------------------------

def wrap90(angle: float) -> float:
    """A cube looks the same every 90 degrees: wrap into [-45, 45)."""
    return (angle + 45.0) % 90.0 - 45.0


def detect_cubes(rgb, color_ranges, min_frac=0.001, max_frac=0.06, min_fill=0.6):
    """Cubes in one RGB frame, as dicts with u, v (fractions of the width),
    angle (degrees, wrapped), area_frac, color and box (pixels).

    A cube is a blob in one colour range whose area is between ``min_frac``
    and ``max_frac`` of the picture, that fills most of its rotated
    rectangle (``min_fill``) and does not touch the picture edge.
    """
    import cv2

    from unoq_braccio_driver.color_vision import to_hsv

    hsv = to_hsv(rgb)
    height, width = hsv.shape[:2]
    total = float(height * width)
    cubes = []
    for color, ranges in color_ranges.items():
        mask = None
        for low, high in ranges:
            part = cv2.inRange(hsv, np.array(low), np.array(high))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if not min_frac * total <= area <= max_frac * total:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1:
                continue  # cut off by the picture edge
            (cx, cy), (rw, rh), angle = cv2.minAreaRect(contour)
            if rw * rh <= 0 or area / (rw * rh) < min_fill:
                continue  # not a square-ish solid blob (e.g. the arm, a shadow)
            cubes.append({
                "u": cx / width, "v": cy / width, "angle": wrap90(angle),
                "area_frac": area / total, "color": color,
                "box": [x, y, x + w, y + h], "width": width, "height": height,
            })
    return cubes


def summarise(frames_cubes, min_share=0.7):
    """One steady reading from several frames' ``detect_cubes`` results.

    Needs exactly one cube in at least ``min_share`` of the frames. Returns
    ``(reading, None)`` or ``(None, reason)``.
    """
    singles = [cubes[0] for cubes in frames_cubes if len(cubes) == 1]
    if not frames_cubes:
        return None, "no camera frames"
    if len(singles) < min_share * len(frames_cubes):
        counts = sorted({len(c) for c in frames_cubes})
        if counts == [0]:
            return None, "no cube seen"
        return None, f"need exactly ONE cube in view (saw {counts})"
    colors = [c["color"] for c in singles]
    angle4 = [math.radians(4.0 * c["angle"]) for c in singles]
    mean4 = math.degrees(math.atan2(sum(math.sin(a) for a in angle4),
                                    sum(math.cos(a) for a in angle4)))
    reading = {
        "u": float(np.median([c["u"] for c in singles])),
        "v": float(np.median([c["v"] for c in singles])),
        "angle": wrap90(mean4 / 4.0),
        "area_frac": float(np.median([c["area_frac"] for c in singles])),
        "color": max(set(colors), key=colors.count),
        "width": singles[0]["width"], "height": singles[0]["height"],
        "jitter": float(max(np.ptp([c["u"] for c in singles]),
                            np.ptp([c["v"] for c in singles])) * singles[0]["width"]),
    }
    return reading, None


def steady_cubes(frames_cubes, min_share=0.6, radius=0.03):
    """Every cube seen in at least ``min_share`` of the frames, each averaged
    like :func:`summarise`. For tables with several cubes."""
    if not frames_cubes:
        return []
    seeds = max(frames_cubes, key=len)
    steady = []
    for seed in seeds:
        track = [cube_near(cubes, seed["u"], seed["v"], radius) for cubes in frames_cubes]
        track = [c for c in track if c is not None]
        if len(track) >= min_share * len(frames_cubes):
            reading, _ = summarise([[c] for c in track], min_share=1.0)
            steady.append(reading)
    return steady


def cube_near(cubes, u, v, radius=0.04):
    """The detected cube closest to (u, v) within ``radius`` (width fractions), or None."""
    near = [c for c in cubes if math.hypot(c["u"] - u, c["v"] - v) <= radius]
    return min(near, key=lambda c: math.hypot(c["u"] - u, c["v"] - v)) if near else None


# -- regressors (inputs and outputs already standardised) --------------------------

class Knn:
    """Blend of the nearest examples, closer ones counting more."""

    kind = "knn"
    min_examples = 1

    def __init__(self, k=4):
        self.k = k

    def clone(self):
        return Knn(self.k)

    def fit(self, x, y):
        self.x, self.y = np.asarray(x, float), np.asarray(y, float)
        return self

    def predict(self, x):
        out = []
        for row in np.atleast_2d(x):
            dist = np.linalg.norm(self.x - row, axis=1)
            order = np.argsort(dist)[: self.k]
            weights = 1.0 / (dist[order] + 1e-3) ** 2
            out.append(weights @ self.y[order] / weights.sum())
        return np.array(out)

    def to_dict(self):
        return {"k": self.k, "x": self.x.tolist(), "y": self.y.tolist()}

    @classmethod
    def from_dict(cls, d):
        return cls(d["k"]).fit(d["x"], d["y"])


def _poly_features(x, degree):
    u, v = x[:, 0], x[:, 1]
    cols = [u ** i * v ** j for total in range(degree + 1)
            for i in range(total + 1) for j in [total - i]]
    return np.stack(cols, axis=1)


class Poly:
    """Smooth polynomial surface in (u, v), lightly regularised."""

    kind = "poly"

    def __init__(self, degree=2, ridge=1e-2):
        self.degree, self.ridge = degree, ridge
        self.min_examples = (degree + 1) * (degree + 2) // 2 + 3

    def clone(self):
        return Poly(self.degree, self.ridge)

    def fit(self, x, y):
        f = _poly_features(np.asarray(x, float), self.degree)
        reg = self.ridge * np.eye(f.shape[1])
        reg[0, 0] = 0.0  # do not shrink the constant
        self.coef = np.linalg.solve(f.T @ f + reg, f.T @ np.asarray(y, float))
        return self

    def predict(self, x):
        return _poly_features(np.atleast_2d(np.asarray(x, float)), self.degree) @ self.coef

    def to_dict(self):
        return {"degree": self.degree, "ridge": self.ridge, "coef": self.coef.tolist()}

    @classmethod
    def from_dict(cls, d):
        model = cls(d["degree"], d["ridge"])
        model.coef = np.array(d["coef"])
        return model


class Mlp:
    """Small neural network: one hidden tanh layer, trained with Adam."""

    kind = "mlp"
    min_examples = 12

    def __init__(self, hidden=16, decay=1e-3, epochs=3000, rate=0.01, seed=0):
        self.hidden, self.decay, self.epochs, self.rate, self.seed = hidden, decay, epochs, rate, seed

    def clone(self):
        return Mlp(self.hidden, self.decay, self.epochs, self.rate, self.seed)

    def fit(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        rng = np.random.default_rng(self.seed)
        params = [rng.normal(0, 1.0 / math.sqrt(x.shape[1]), (x.shape[1], self.hidden)),
                  np.zeros(self.hidden),
                  rng.normal(0, 1.0 / math.sqrt(self.hidden), (self.hidden, y.shape[1])),
                  np.zeros(y.shape[1])]
        m = [np.zeros_like(p) for p in params]
        s = [np.zeros_like(p) for p in params]
        for step in range(1, self.epochs + 1):
            w1, b1, w2, b2 = params
            h = np.tanh(x @ w1 + b1)
            err = (h @ w2 + b2 - y) / len(x)
            gh = (err @ w2.T) * (1 - h ** 2)
            grads = [x.T @ gh + self.decay * w1, gh.sum(0), h.T @ err + self.decay * w2, err.sum(0)]
            for i, g in enumerate(grads):
                m[i] = 0.9 * m[i] + 0.1 * g
                s[i] = 0.999 * s[i] + 0.001 * g * g
                params[i] -= self.rate * (m[i] / (1 - 0.9 ** step)) / (
                    np.sqrt(s[i] / (1 - 0.999 ** step)) + 1e-8)
        self.params = params
        return self

    def predict(self, x):
        w1, b1, w2, b2 = self.params
        return np.tanh(np.atleast_2d(np.asarray(x, float)) @ w1 + b1) @ w2 + b2

    def to_dict(self):
        return {"hidden": self.hidden, "decay": self.decay, "params": [p.tolist() for p in self.params]}

    @classmethod
    def from_dict(cls, d):
        model = cls(d["hidden"], d["decay"])
        model.params = [np.array(p) for p in d["params"]]
        return model


REGRESSORS = {"knn": Knn, "poly": Poly, "mlp": Mlp}


def candidates():
    """Every model tried at each refit; the one that predicts best wins."""
    return [Knn(4), Poly(1), Poly(2), Poly(3), Mlp()]


def _label(model):
    return f"{model.kind}{model.degree}" if model.kind == "poly" else model.kind


# -- training data ----------------------------------------------------------------

def _inputs(examples):
    return np.array([[e["u"], e["v"]] for e in examples], float)


def _roll_targets(examples):
    """Wrist roll is learned as ``roll = sign * cube_angle + offset(u, v)``.

    The offset follows the base angle (so the position), and the sign
    depends on whether the camera is mirrored relative to the wrist, so both
    are tried and the steadier one is kept. Offsets are unwrapped around
    their circular mean (a cube repeats every 90 degrees).
    """
    best = None
    for sign in (1, -1):
        raw = [wrap90(e["grab"][ROLL] - sign * e["angle"]) for e in examples]
        rad = [math.radians(4 * r) for r in raw]
        mean = math.degrees(math.atan2(sum(map(math.sin, rad)), sum(map(math.cos, rad)))) / 4
        offsets = [mean + wrap90(r - mean) for r in raw]
        spread = float(np.std(offsets))
        if best is None or spread < best[0] - 1e-9:
            best = (spread, sign, offsets)
    return best[1], best[2]


def _targets(examples, sign):
    _, offsets = sign
    return np.array([list(e["above"][:ARM]) + list(e["grab"][:ARM]) + [o]
                     for e, o in zip(examples, offsets)], float)


def cross_validate(model, examples, folds=5):
    """Mean absolute error in degrees on examples the model did not see."""
    n = len(examples)
    if n < 2:
        return float("inf")
    groups = [list(range(i, n, min(folds, n))) for i in range(min(folds, n))]
    errors = []
    for held in groups:
        train = [examples[i] for i in range(n) if i not in held]
        if len(train) < model.min_examples:
            return float("inf")
        fitted = PickModel.fit(train, model)
        for i in held:
            pred = fitted.predict(examples[i])
            truth = examples[i]
            errors += [abs(pred["above"][j] - truth["above"][j]) for j in range(ARM)]
            errors += [abs(pred["grab"][j] - truth["grab"][j]) for j in range(ARM)]
    return float(np.mean(errors))


class PickModel:
    """Camera reading -> above / grab / lift poses, plus how sure it is."""

    def __init__(self, regressor, x_mean, x_std, y_mean, y_std, roll_sign,
                 train_uv, gripper_open, gripper_closed, info=None):
        self.regressor = regressor
        self.x_mean, self.x_std = np.asarray(x_mean), np.asarray(x_std)
        self.y_mean, self.y_std = np.asarray(y_mean), np.asarray(y_std)
        self.roll_sign = roll_sign
        self.train_uv = np.asarray(train_uv, float)
        self.gripper_open, self.gripper_closed = int(gripper_open), int(gripper_closed)
        self.info = info or {}

    @classmethod
    def fit(cls, examples, regressor):
        x = _inputs(examples)
        sign, offsets = _roll_targets(examples)
        y = _targets(examples, (sign, offsets))
        x_mean, x_std = x.mean(0), np.maximum(x.std(0), 1e-3)
        y_mean, y_std = y.mean(0), np.maximum(y.std(0), 1e-3)
        fresh = regressor.clone()
        fresh.fit((x - x_mean) / x_std, (y - y_mean) / y_std)
        return cls(fresh, x_mean, x_std, y_mean, y_std, sign, x,
                   np.median([e["grab"][GRIP] for e in examples]),
                   np.median([e["closed"] for e in examples]))

    def nearest(self, reading):
        """Distance from the reading to the closest example, in pixels at 640 wide."""
        d = np.linalg.norm(self.train_uv - [reading["u"], reading["v"]], axis=1).min()
        return float(d * 640.0)

    def predict(self, reading):
        x = (np.array([[reading["u"], reading["v"]]]) - self.x_mean) / self.x_std
        y = self.regressor.predict(x)[0] * self.y_std + self.y_mean
        roll = self.roll_sign * reading["angle"] + y[2 * ARM]
        roll -= 90.0 * round((roll - 90.0) / 90.0)  # equivalent roll closest to centred
        from unoq_braccio_driver.braccio_model import JOINT_LIMITS, JOINT_NAMES

        def pose(values, gripper):
            full = list(values) + [roll, gripper]
            return [int(round(max(JOINT_LIMITS[n].minimum, min(JOINT_LIMITS[n].maximum, v))))
                    for n, v in zip(JOINT_NAMES, full)]

        above = pose(y[:ARM], self.gripper_open)
        return {
            "above": above,
            "grab": pose(y[ARM:2 * ARM], self.gripper_open),
            "grip": pose(y[ARM:2 * ARM], self.gripper_closed),
            "lift": above[:GRIP] + [self.gripper_closed],
            "nearest_px": self.nearest(reading),
        }

    def to_dict(self):
        return {
            "version": 1, "kind": self.regressor.kind, "regressor": self.regressor.to_dict(),
            "x_mean": self.x_mean.tolist(), "x_std": self.x_std.tolist(),
            "y_mean": self.y_mean.tolist(), "y_std": self.y_std.tolist(),
            "roll_sign": self.roll_sign, "train_uv": self.train_uv.tolist(),
            "gripper_open": self.gripper_open, "gripper_closed": self.gripper_closed,
            **self.info,
        }

    @classmethod
    def from_dict(cls, d):
        info = {k: v for k, v in d.items() if k not in (
            "version", "kind", "regressor", "x_mean", "x_std", "y_mean", "y_std",
            "roll_sign", "train_uv", "gripper_open", "gripper_closed")}
        return cls(REGRESSORS[d["kind"]].from_dict(d["regressor"]), d["x_mean"], d["x_std"],
                   d["y_mean"], d["y_std"], d["roll_sign"], d["train_uv"],
                   d["gripper_open"], d["gripper_closed"], info)


def train(examples):
    """Try every candidate, keep the one with the lowest cross-validated
    error, refit it on all examples. Returns ``(model, scores)``."""
    scores = {}
    best = None
    for model in candidates():
        if len(examples) < model.min_examples:
            continue
        err = cross_validate(model, examples)
        scores[_label(model)] = err
        if best is None or err < best[0]:
            best = (err, model)
    if best is None:
        raise ValueError("need at least 1 example")
    fitted = PickModel.fit(examples, best[1])
    fitted.info = {"examples": len(examples), "chosen": _label(best[1]),
                   "error_deg": None if math.isinf(best[0]) else round(best[0], 2),
                   "scores": {k: (None if math.isinf(v) else round(v, 2)) for k, v in scores.items()},
                   "created": time.strftime("%Y-%m-%d %H:%M:%S")}
    return fitted, scores


# -- sessions on disk --------------------------------------------------------------

class Session:
    """One camera + table setup: its examples, drop poses, tests and models.

    ``~/.ros/braccio_teach/<name>/`` holds session.yaml, reference.jpg,
    examples.jsonl, drops.yaml, tests.jsonl, paths/ and models/.
    """

    def __init__(self, name, root=TEACH_ROOT):
        self.name = name
        self.dir = os.path.join(os.path.expanduser(root), name)
        os.makedirs(os.path.join(self.dir, "models"), exist_ok=True)
        os.makedirs(os.path.join(self.dir, "paths"), exist_ok=True)

    def path(self, *parts):
        return os.path.join(self.dir, *parts)

    # info + reference picture
    def info(self):
        import yaml

        if not os.path.exists(self.path("session.yaml")):
            return {}
        with open(self.path("session.yaml"), encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}

    def save_info(self, info):
        import yaml

        with open(self.path("session.yaml"), "w", encoding="utf-8") as handle:
            yaml.safe_dump(info, handle, sort_keys=False)

    def reference(self):
        import cv2

        if not os.path.exists(self.path("reference.jpg")):
            return None
        image = cv2.imread(self.path("reference.jpg"))
        return None if image is None else image[:, :, ::-1]

    def save_reference(self, rgb):
        import cv2

        cv2.imwrite(self.path("reference.jpg"), np.ascontiguousarray(rgb[:, :, ::-1]))

    # examples
    def examples(self):
        return _read_jsonl(self.path("examples.jsonl"))

    def add_example(self, example):
        _append_jsonl(self.path("examples.jsonl"), example)

    def remove_last_example(self):
        examples = self.examples()
        if not examples:
            return None
        with open(self.path("examples.jsonl"), "w", encoding="utf-8") as handle:
            for e in examples[:-1]:
                handle.write(json.dumps(e) + "\n")
        return examples[-1]

    def log_test(self, record):
        _append_jsonl(self.path("tests.jsonl"), record)

    def tests(self):
        return _read_jsonl(self.path("tests.jsonl"))

    def save_path(self, example_id, samples):
        with open(self.path("paths", f"{example_id}.json"), "w", encoding="utf-8") as handle:
            json.dump(samples, handle)

    # drop poses
    def drops(self):
        import yaml

        if not os.path.exists(self.path("drops.yaml")):
            return {}
        with open(self.path("drops.yaml"), encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}

    def save_drops(self, drops):
        import yaml

        with open(self.path("drops.yaml"), "w", encoding="utf-8") as handle:
            yaml.safe_dump(drops, handle, sort_keys=True)

    # models
    def save_model(self, model):
        name = f"model_{model.info['examples']:04d}"
        model.info["name"] = name
        for filename in (f"{name}.json", "latest.json"):
            with open(self.path("models", filename), "w", encoding="utf-8") as handle:
                json.dump(model.to_dict(), handle, indent=1)
        return name

    def models(self):
        return sorted(f[:-5] for f in os.listdir(self.path("models"))
                      if f.startswith("model_") and f.endswith(".json"))

    def load_model(self, name=""):
        """A saved model by name (``model_0010``), or the latest; None if none yet."""
        filename = f"{name or 'latest'}.json"
        if not filename.startswith(("model_", "latest")):
            filename = f"model_{name}.json"
        path = self.path("models", filename)
        if not os.path.exists(path):
            return None
        with open(path, encoding="utf-8") as handle:
            return PickModel.from_dict(json.load(handle))


def _read_jsonl(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def _append_jsonl(path, record):
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(json.dumps(record) + "\n")


# -- has the camera moved? ------------------------------------------------------

def compare_to_reference(rgb, reference):
    """How the camera view moved since the session's reference picture.

    Returns dict(shift_px, scale, rotation_deg, matches) measured at 640 px
    wide, or None when the pictures cannot be matched (very different view).
    Cubes and the arm moving around are ignored by the robust fit.
    """
    import cv2

    def gray(image):
        g = cv2.cvtColor(np.ascontiguousarray(image), cv2.COLOR_RGB2GRAY)
        scale = 640.0 / g.shape[1]
        return cv2.resize(g, (640, int(round(g.shape[0] * scale))))

    a, b = gray(reference), gray(rgb)
    orb = cv2.ORB_create(1500)
    ka, da = orb.detectAndCompute(a, None)
    kb, db = orb.detectAndCompute(b, None)
    if da is None or db is None:
        return None
    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(da, db)
    if len(matches) < 12:
        return None
    src = np.float32([ka[m.queryIdx].pt for m in matches])
    dst = np.float32([kb[m.trainIdx].pt for m in matches])
    transform, inliers = cv2.estimateAffinePartial2D(src, dst, ransacReprojThreshold=3.0)
    if transform is None or inliers is None or inliers.sum() < 10:
        return None
    scale = float(math.hypot(transform[0, 0], transform[1, 0]))
    centre = np.array([320.0, a.shape[0] / 2.0])
    moved = transform[:, :2] @ centre + transform[:, 2]
    return {
        "shift_px": float(np.linalg.norm(moved - centre)),
        "scale": scale,
        "rotation_deg": float(math.degrees(math.atan2(transform[1, 0], transform[0, 0]))),
        "matches": int(inliers.sum()),
    }


def camera_moved(check, shift_px=8.0, scale=0.02, rotation_deg=1.5):
    """True when ``compare_to_reference`` says the view changed enough to matter."""
    return (check is None or check["shift_px"] > shift_px
            or abs(check["scale"] - 1.0) > scale or abs(check["rotation_deg"]) > rotation_deg)
