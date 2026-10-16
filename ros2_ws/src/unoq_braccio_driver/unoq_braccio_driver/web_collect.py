"""Auto-collect: many kNN examples from one taught example, by turning the base step by step.

The web dashboard's Collect tab. One taught example (the seed) is a posture
that puts the fingers around a cube on the table. Turning only the base
keeps that posture at exactly the same height, so the same posture at every
base angle between the two ends set on the page reaches the table too. At
each base step the arm puts the cube down there, steps out of the camera's
view, reads where the cube landed, picks it back up with that same pose and,
if the cube really left the table, saves the example (kind "sweep"). No arm
geometry is used: only what the camera saw and the pose the arm used, as for
every other kNN example. One seed fills one arc; a seed taught nearer or
further out fills another. "Sweep all" takes every ticked hand-taught
example in the order they were taught (record a straight line from the base
outwards, a few cm apart) and sweeps each one's arc in turn, so a handful of
taught examples becomes a grid over the whole table.

Runs like web_teach.Teacher (same session files, ask/answer with the page,
Stop freezes the arm); a person only presses OK to hand over the cube or put
it back when it is lost. Settings (base ends, step) live in the session's
collect.yaml.
"""

from __future__ import annotations

import time
import uuid

from unoq_braccio_driver import braccio_workspace as ws
from unoq_braccio_driver import pick_learning as pl
from unoq_braccio_driver.web_teach import UNSURE_PX, Stopped, Teacher

STEPS = (3, 5, 10)              # base degrees between collected examples
SAME_POSTURE_DEG = 3            # an existing example this close in posture and base: spot already done


def plan_bases(seed_base, base_min, base_max, step, done=()):
    """Base angles to visit from ``base_min`` to ``base_max`` every ``step``, in order,
    leaving out the seed's own angle and angles already in ``done``."""
    lo, hi = sorted((int(base_min), int(base_max)))
    bases, b = [], lo
    while b <= hi:
        if abs(b - seed_base) >= step / 2.0 and all(abs(b - d) >= step / 2.0 for d in done):
            bases.append(b)
        b += step
    return bases


def posed(pose, base):
    """``pose`` turned to ``base``: nothing else changes, so the height stays the same."""
    return [int(base)] + list(pose[1:])


class Collector(Teacher):
    kind = "collect"

    def __init__(self, arm, session_name, **kwargs):
        super().__init__(arm, session_name, **kwargs)
        self.settings = self.load_settings()
        self.seed_id = None
        self.progress = {"done": 0, "total": 0, "saved": 0, "skipped": 0, "base": None, "arc": 0, "arcs": 0}
        self.holding = False
        self.skip_arc = False

    # -- settings ------------------------------------------------------------------

    def seeds(self):
        """Hand-taught examples that can be swept (newest first)."""
        return [e for e in reversed(self.examples) if e.get("kind", "teach") in ("teach", "correction")][:30]

    def line(self):
        """The ticked hand-taught examples for "Sweep all", in the order they were taught
        (none ticked = all of them)."""
        chosen = set(self.settings["chosen"])
        return [e for e in reversed(self.seeds()) if not chosen or e.get("id") in chosen]

    def seed(self):
        seeds = self.seeds()
        return next((e for e in seeds if e.get("id") == self.seed_id), seeds[0] if seeds else None)

    def load_settings(self):
        import yaml

        try:
            with open(self.session.path("collect.yaml"), encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, ValueError):
            data = {}
        return {"base_min": data.get("base_min"), "base_max": data.get("base_max"),
                "step": data.get("step", 5) if data.get("step", 5) in STEPS else 5,
                "mode": data.get("mode") if data.get("mode") in ("one", "all") else "one",
                "chosen": [str(i) for i in data.get("chosen") or []]}

    def save_settings(self):
        import yaml

        with open(self.session.path("collect.yaml"), "w", encoding="utf-8") as handle:
            yaml.safe_dump(self.settings, handle, sort_keys=False)

    def update(self, payload, base_now):
        """A change from the page: set an end at the base angle now, the step, or the seed."""
        if payload.get("set_end") == "left":       # a bigger base angle turns the arm left
            self.settings["base_max"] = int(base_now)
        elif payload.get("set_end") == "right":
            self.settings["base_min"] = int(base_now)
        if payload.get("step") in STEPS:
            self.settings["step"] = payload["step"]
        if payload.get("seed"):
            if not any(e.get("id") == payload["seed"] for e in self.seeds()):
                return False, "Unknown example."
            self.seed_id = payload["seed"]
        if payload.get("mode") in ("one", "all"):
            self.settings["mode"] = payload["mode"]
        if payload.get("toggle"):             # tick / untick an example for "Sweep all"
            ids = [e.get("id") for e in self.seeds()]
            if payload["toggle"] not in ids:
                return False, "Unknown example."
            chosen = set(self.settings["chosen"] or ids)
            chosen ^= {payload["toggle"]}
            self.settings["chosen"] = [i for i in ids if i in chosen] if chosen != set(ids) else []
            if not self.line():
                return False, "Keep at least one example ticked."
        if payload.get("skip_arc"):
            self.skip_arc = True
            return True, ""
        self.save_settings()
        return True, ""

    def window(self, seed=None):
        """(min, max) base for the sweep: the ends set on the page, else 30 degrees either side of the seed."""
        seed = seed or self.seed()
        middle = seed["grab"][0] if seed else 90
        lo = self.settings["base_min"] if self.settings["base_min"] is not None else max(0, middle - 30)
        hi = self.settings["base_max"] if self.settings["base_max"] is not None else min(180, middle + 30)
        return min(lo, hi), max(lo, hi)

    def planned(self, seed=None):
        """Base angles still to collect for ``seed`` (default: the chosen one)."""
        seed = seed or self.seed()
        if seed is None:
            return []
        done = [e["grab"][0] for e in self.examples
                if max(abs(a - b) for a, b in zip(e["grab"][1:4], seed["grab"][1:4])) <= SAME_POSTURE_DEG]
        lo, hi = self.window(seed)
        return plan_bases(seed["grab"][0], lo, hi, self.settings["step"], done)

    def to_sweep(self):
        """The examples this run will sweep: the chosen one, or every ticked one in taught order."""
        if self.settings["mode"] == "all":
            return self.line()
        seed = self.seed()
        return [seed] if seed else []

    def state(self):
        state = super().state()
        seed = self.seed()
        lo, hi = self.window()
        with self.lock:
            state.update({
                "kind": self.kind,
                "collect": {
                    "settings": dict(self.settings), "window": [lo, hi],
                    "planned": sum(len(self.planned(e)) for e in self.to_sweep()),
                    "seed": None if seed is None else seed.get("id"),
                    "seeds": [{"id": e.get("id"), "color": e.get("color"), "base": e["grab"][0],
                               "x": round(e["u"] * 640), "y": round(e["v"] * 640),
                               "ticked": e.get("id") in [s.get("id") for s in self.line()],
                               "todo": len(self.planned(e))} for e in self.seeds()],
                    "progress": dict(self.progress), "steps": list(STEPS),
                },
                # every example's spot, so the camera can show how much of the table is covered
                "spots": [[round(e["u"] * 640), round(e["v"] * 640), e.get("kind", "teach")] for e in self.examples],
            })
        return state

    # -- the flow ----------------------------------------------------------------------

    def menu(self):
        while True:
            try:
                self.stop_flag.clear()
                self.reading = None
                if not self.examples:
                    self.ask("collect_setup", "Teach one example first (Train tab), then come back: "
                             "I will repeat it all along the base.", ["quit"])
                    self.go_home()
                    return
                choice = self.ask("collect_setup",
                                  "Put the cube where an example was taught (or anywhere I can pick it). "
                                  "Set the base ends and the step, choose what to repeat, then start.",
                                  ["start", "quit"])
                if choice == "quit":
                    self.release_if_holding()
                    self.go_home()
                    return
                self.sweep_all(self.to_sweep())
            except Stopped:
                self.holding = self.holding and self.arm.target[5] != ws.GRIPPER_OPEN
                self.after_stop()

    def release_if_holding(self):
        if self.holding:
            self.ask("collect_give_back", "I am still holding the cube. Hold your hand under it and press OK.",
                     ["ok"])
            self.move(list(self.arm.target[:5]) + [ws.GRIPPER_OPEN], settle=0.8)
            self.holding = False

    def acquire(self):
        """Get the cube into the gripper: pick it if the model knows where it is, else a hand-over.
        False when the person chose to stop."""
        self.go_home()
        self.busy("reading", "Looking for the cube…")
        reading, _ = self.read_cube()
        if reading is not None and self.model is not None:
            guess = self.model.predict(reading)
            if guess["nearest_px"] <= UNSURE_PX:
                self.busy("moving", f"Picking up the {reading['color']} cube…")
                self.move(guess["above"])
                self.move(guess["grab"])
                self.move(guess["grip"], settle=0.8)
                self.move(guess["lift"])
                self.go_home(gripper=ws.GRIPPER_CLOSED)
                if self.cube_gone(reading):
                    self.holding = True
                    return True
                self.move(list(self.arm.target[:5]) + [ws.GRIPPER_OPEN], settle=0.5)
                self.go_home()
        self.move(self.home(), settle=0.3)
        if self.ask("collect_handover", "Put the cube between the open fingers and press OK.",
                    ["ok", "quit"]) == "quit":
            return False
        self.move(self.home(ws.GRIPPER_CLOSED), settle=1.0)
        self.holding = True
        return True

    def cube_gone(self, reading):
        frames = self.arm.fresh_frames(5)
        left = [pl.cube_near(self.detect(f), reading["u"], reading["v"]) for f in frames]
        return sum(c is None for c in left) > len(left) / 2

    def sweep_all(self, seeds):
        """Sweep each seed's arc in turn (taught order: the line from the base outwards)."""
        plans = [(seed, self.planned(seed)) for seed in seeds]
        plans = [(seed, bases) for seed, bases in plans if bases]
        self.progress = {"done": 0, "total": sum(len(b) for _, b in plans), "saved": 0, "skipped": 0,
                         "base": None, "arc": 0, "arcs": len(plans)}
        if not plans:
            self.event("warn", "Nothing new to collect: widen the ends, or teach another example.")
            return
        self.event("info", f"Collecting {self.progress['total']} spots on {len(plans)} arc(s).")
        if not self.holding and not self.acquire():
            return
        last = plans[0][0]
        for number, (seed, bases) in enumerate(plans, start=1):
            self.progress["arc"] = number
            self.skip_arc = False
            last = seed
            if not self.sweep(seed, bases):
                break
            self.refit()
            self.save_model()
        self.event("picked", f"Done: {self.progress['saved']} new examples, {self.progress['skipped']} skipped.")
        self.put_back(last)

    def put_back(self, seed):
        """Put a held cube down where ``seed`` was taught, and go home."""
        closed, opened = seed["closed"], ws.GRIPPER_OPEN
        if self.holding:
            self.busy("moving", "Putting the cube back…")
            self.move([seed["grab"][0]] + list(self.home()[1:5]) + [closed])
            self.move(seed["above"][:5] + [closed])
            self.move(seed["grab"][:5] + [closed], settle=0.4)
            self.move(seed["grab"][:5] + [opened], settle=0.6)
            self.holding = False
            self.move(seed["above"][:5] + [opened])
        self.go_home()

    def sweep(self, seed, bases):
        """One arc: ``seed``'s posture at each base angle. False when the person stopped it."""
        closed, opened = seed["closed"], ws.GRIPPER_OPEN
        for base in bases:
            if self.skip_arc:
                self.skip_arc = False
                self.event("info", f"Skipped the rest of arc {self.progress['arc']}.")
                self.progress["done"] += len(bases) - bases.index(base)
                return True
            self.progress["base"] = base
            above, grab = posed(seed["above"], base), posed(seed["grab"], base)
            # 1. put the cube down with the seed's posture, turned to this base angle
            self.busy("moving", f"Putting the cube down at {base}°…")
            self.move([base] + list(self.home()[1:5]) + [closed])       # turn while upright
            self.move(above[:5] + [closed])
            self.move(grab[:5] + [closed], settle=0.4)
            self.move(grab[:5] + [opened], settle=0.6)
            self.holding = False
            self.move(above[:5] + [opened])
            self.go_home()
            # 2. where did it land?
            self.busy("reading", "Where did it land?")
            reading, why = self.read_cube()
            self.progress["done"] += 1
            if reading is None:
                self.progress["skipped"] += 1
                self.event("warn", f"Cannot see the cube after putting it down at {base}° ({why}).")
                if self.ask("collect_lost", "I lost the cube. Put it back on the table where I can see it "
                            "(or between my fingers) and press OK.", ["ok", "quit"]) == "quit":
                    return False
                if not self.acquire():
                    return False
                continue
            self.reading = {"color": reading["color"], "x": round(reading["u"] * reading["width"]),
                            "y": round(reading["v"] * reading["width"]), "angle": round(reading["angle"]),
                            "jitter": round(reading["jitter"], 1)}
            # 3. pick it back up with the very same pose: the proof the example is right
            self.busy("testing", "Picking it back up to check…")
            self.move(above[:5] + [opened])
            self.move(grab[:5] + [opened])
            self.move(grab[:5] + [closed], settle=0.8)
            self.move(above[:5] + [closed])
            self.go_home(gripper=closed)
            if not self.cube_gone(reading):
                self.progress["skipped"] += 1
                self.event("missed", f"Could not pick it back up at {base}°: not saved.")
                self.move(self.home(), settle=0.5)
                if not self.acquire():
                    return False
                continue
            self.holding = True
            example = {
                "id": time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:4],
                "time": time.strftime("%Y-%m-%d %H:%M:%S"), "kind": "sweep", "seed": seed.get("id"),
                **{k: reading[k] for k in ("u", "v", "angle", "area_frac", "color", "width", "height", "jitter")},
                "above": above[:5] + [opened], "grab": grab[:5] + [opened], "closed": closed,
                "lift": above[:5] + [closed],
            }
            self.session.add_example(example)
            self.examples.append(example)
            self.progress["saved"] += 1
            self.event("saved", f"Example {len(self.examples)} saved at {base}°!")
            if self.progress["saved"] % 5 == 0:
                self.refit()
        return True
