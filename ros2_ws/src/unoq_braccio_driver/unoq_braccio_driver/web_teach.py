"""Teach the arm to pick from the web dashboard: teach_pick's flow, driven by buttons.

It writes the same sessions as teach_pick (``~/.ros/braccio_teach/<session>/``:
examples, tests, drop poses, models), so a session can be taught in the
terminal one day and in the browser the next.

A worker thread runs the flow and stops at each step to wait for the page:
``state()`` says which step it is on and which answers it takes, and
``answer(choice)`` gives one. The joints are jogged with the dashboard's
usual move controls; whatever the arm holds when the operator presses Next is
what gets recorded. ``interrupt()`` (the Stop button) freezes the arm and
returns to the menu; ``quit()`` ends teaching.

No ROS here: ``arm`` is anything with ``send(pose)``, ``freeze()``,
``servo``, ``target``, ``latest_frame()`` and ``fresh_frames(count)``, so
the flow can be tested without a robot.
"""

from __future__ import annotations

import queue
import threading
import time
import uuid

from unoq_braccio_driver import braccio_workspace as ws
from unoq_braccio_driver import pick_learning as pl
from unoq_braccio_driver.braccio_model import START_POSE

SPEED_DEG_S = 60.0      # as serial_bridge's default; only sets move timeouts
FRAMES = 10             # camera frames averaged per reading
TEST_EVERY = 5          # examples between offered tests (as teach_pick)
UNSURE_PX = 60.0        # further than this from every example: not learned there
JOG_PHASES = ("above", "grab", "close", "drop")


class Stopped(Exception):
    """Stop was pressed: freeze and go back to the menu."""


class Quit(Exception):
    """Teaching was ended."""


class Teacher:
    def __init__(self, arm, session_name, root=pl.TEACH_ROOT, setup_file="", log=None):
        self.arm = arm
        self.session = pl.Session(session_name, root=root)
        self.setup_file = setup_file
        self.log = log or (lambda line: None)
        self.examples = self.session.examples()
        self.model = self.session.load_model()
        self.saved_at = self.model.info.get("examples", 0) if self.model else 0
        self.success = self.success_text()

        self.lock = threading.Lock()
        self.answers = queue.Queue()
        self.stop_flag = threading.Event()
        self.quit_flag = threading.Event()
        self.phase, self.prompt, self.choices = "starting", "Getting ready…", []
        self.reading = None
        self.review = []
        self.events = []        # {"seq", "kind", "text"}: the page toasts / celebrates new ones
        self.seq = 0
        self.path_log = []
        self.done = False
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    # -- talking to the page ---------------------------------------------------

    def state(self):
        model = None
        if self.model is not None:
            model = {k: self.model.info.get(k) for k in ("name", "chosen", "error_deg", "examples")}
        with self.lock:
            return {
                "session": self.session.name, "phase": self.phase, "prompt": self.prompt,
                "choices": list(self.choices), "examples": len(self.examples), "model": model,
                "scores": (self.model.info.get("scores") if self.model else None) or {},
                "success": self.success, "reading": self.reading, "review": list(self.review),
                "events": self.events[-6:], "done": self.done,
            }

    def answer(self, choice):
        """Give the step waiting now its answer. Returns (ok, message)."""
        if self.done:
            return False, "Teaching has ended."
        with self.lock:
            if choice not in self.choices:
                return False, "That is not an option right now."
        self.answers.put(choice)
        return True, ""

    def interrupt(self):
        self.stop_flag.set()

    def quit(self):
        self.quit_flag.set()

    def on_jog(self, pose):
        """The operator moved a joint; kept as the example's path, like teach_pick."""
        if self.phase in JOG_PHASES:
            self.path_log.append([round(time.time(), 2)] + list(pose))

    def event(self, kind, text):
        with self.lock:
            self.seq += 1
            self.events.append({"seq": self.seq, "kind": kind, "text": text})
            del self.events[:-20]
        self.log(f"teach: {text}")

    def busy(self, phase, prompt):
        with self.lock:
            self.phase, self.prompt, self.choices = phase, prompt, []

    def ask(self, phase, prompt, choices):
        """Show a step and wait for one of ``choices`` from the page."""
        while not self.answers.empty():
            self.answers.get_nowait()
        with self.lock:
            self.phase, self.prompt, self.choices = phase, prompt, list(choices)
        while True:
            self.check()
            try:
                choice = self.answers.get(timeout=0.2)
            except queue.Empty:
                continue
            if choice in choices:
                with self.lock:
                    self.choices = []
                return choice

    def check(self):
        if self.quit_flag.is_set():
            raise Quit()
        if self.stop_flag.is_set():
            raise Stopped()

    # -- the arm and camera ------------------------------------------------------

    def home(self, gripper=None):
        pose = list(START_POSE[:5]) + [ws.GRIPPER_OPEN]
        if gripper is not None:
            pose[5] = gripper
        return pose

    def move(self, pose, settle=0.2):
        """Send ``pose`` and wait until the arm reports it is there (Stop-aware)."""
        start = list(self.arm.servo or self.arm.target or pose)
        pose = self.arm.send(pose)
        end = time.monotonic() + max(abs(a - b) for a, b in zip(start, pose)) / SPEED_DEG_S * 1.5 + 2.0
        while time.monotonic() < end:
            self.check()
            servo = self.arm.servo
            if servo is not None and max(abs(a - b) for a, b in zip(servo, pose)) <= 2:
                break
            time.sleep(0.05)
        end = time.monotonic() + settle
        while time.monotonic() < end:
            self.check()
            time.sleep(0.05)

    def go_home(self, gripper=None):
        self.move(self.home(gripper))

    def release_and_home(self, above, grab=None):
        """Put down whatever is held (at ``grab`` if given), open and go home."""
        if grab is not None:
            self.move(list(grab[:5]) + [self.arm.target[5]])
        self.move(list(self.arm.target[:5]) + [ws.GRIPPER_OPEN], settle=0.5)
        self.move(list(above[:5]) + [ws.GRIPPER_OPEN])
        self.go_home()

    def detect(self, rgb):
        return pl.detect_cubes(rgb, ws.CUBE_HSV)

    def read_cube(self):
        frames = self.arm.fresh_frames(FRAMES)
        return pl.summarise([self.detect(f) for f in frames])

    def wait_ready(self, timeout=30.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.check()
            if self.arm.servo is not None and self.arm.latest_frame() is not None:
                return True
            time.sleep(0.1)
        return False

    # -- models ----------------------------------------------------------------

    def refit(self):
        if not self.examples:
            self.model = None
            return
        self.busy("training", f"Training models on {len(self.examples)} examples…")
        self.model, _ = pl.train(self.examples)
        info = self.model.info
        err = info.get("error_deg")
        self.event("trained", f"Trained on {len(self.examples)} examples: {info.get('chosen')} wins"
                   + (f", ~{err:.1f}° error" if err is not None else ""))

    def save_model(self):
        if self.model is not None and len(self.examples) != self.saved_at:
            name = self.session.save_model(self.model)
            self.saved_at = len(self.examples)
            self.event("info", f"Saved {name}.")

    def success_text(self, last=10):
        tests = [t for t in self.session.tests() if t["result"] in ("ok", "fail")][-last:]
        if not tests:
            return ""
        return f"{sum(t['result'] == 'ok' for t in tests)}/{len(tests)}"

    # -- the flow --------------------------------------------------------------

    def run(self):
        try:
            self.busy("starting", "Waiting for the arm and the camera…")
            if not self.wait_ready():
                self.event("error", "No arm position or camera images. Is real.launch.py running?")
                return
            reference = self.session.reference()
            if reference is not None and pl.camera_moved(
                    pl.compare_to_reference(self.arm.latest_frame(), reference)):
                if self.ask("camera_moved", "The camera seems to have moved since this session "
                            "started, so picks may miss. Put it back, or start a new session.",
                            ["carry_on", "quit"]) == "quit":
                    return
            if self.examples and self.model is None:
                self.refit()
            self.go_home()
            self.menu()
        except (Quit, Stopped):
            pass
        except Exception as error:  # noqa: BLE001  (keep the page informed, never hang it)
            self.event("error", f"Teaching stopped: {error}")
        finally:
            self.save_model()
            with self.lock:
                self.done = True
                self.phase, self.prompt, self.choices = "ended", "Teaching ended.", []

    def menu(self):
        actions = {"record": self.record_example, "test": self.test, "drops": self.teach_drops,
                   "review": self.run_review, "undo": self.undo}
        while True:
            try:
                self.stop_flag.clear()
                self.reading = None
                choice = self.ask("menu", "Place ONE cube on the table, then record an example.",
                                  ["record", "test", "drops", "review", "undo", "quit"])
                if choice == "quit":
                    self.go_home()
                    return
                actions[choice]()
            except Stopped:
                self.after_stop()

    def after_stop(self):
        self.arm.freeze()
        self.stop_flag.clear()
        self.event("stopped", "Stopped. The arm is holding still.")
        try:
            if self.ask("stopped", "Stopped. The arm is holding still.", ["home", "stay"]) == "home":
                self.move(list(self.arm.target[:5]) + [ws.GRIPPER_OPEN], settle=0.5)
                self.go_home()
        except Stopped:
            self.arm.freeze()
            self.stop_flag.clear()

    def record_example(self, reading=None, kind="teach"):
        """PLACE -> ABOVE -> GRAB -> CLOSE -> CHECK. True if an example was saved."""
        self.go_home()
        if reading is None:
            self.busy("reading", "Looking at the cube…")
            reading, why = self.read_cube()
            if reading is None:
                self.event("warn", f"Cannot record: {why}.")
                return False
        if not self.examples and self.session.reference() is None:
            self.session.save_reference(self.arm.latest_frame())
            self.session.save_info({
                "created": time.strftime("%Y-%m-%d %H:%M:%S"),
                "image_size": [reading["width"], reading["height"]],
                "setup_file": self.setup_file,
            })
            self.event("info", "Saved the camera reference picture for this session.")
        self.reading = {"color": reading["color"], "x": round(reading["u"] * reading["width"]),
                        "y": round(reading["v"] * reading["width"]), "angle": round(reading["angle"]),
                        "jitter": round(reading["jitter"], 1)}
        self.path_log = []

        guess = self.model.predict(reading) if self.model else None
        if guess:
            start = guess["above"]
        elif self.examples:
            start = list(self.examples[-1]["above"][:5]) + [ws.GRIPPER_OPEN]
        else:
            start = self.home()
        self.busy("moving", "Moving to my best guess…" if guess else "Moving to the start pose…")
        self.move(start)
        if self.ask("above", "Drive the OPEN gripper right above the cube.", ["next", "cancel"]) == "cancel":
            return self.cancel(start)
        above = list(self.arm.target)

        if guess:  # the model's grab, with any roll set while above
            self.busy("moving", "Lowering to my guess…")
            self.move(guess["grab"][:4] + [above[4], above[5]])
        if self.ask("grab", "Lower the open fingers around the cube.", ["next", "cancel"]) == "cancel":
            return self.cancel(above)
        grab = list(self.arm.target)

        if self.ask("close", "Close the gripper on the cube, then lift it a little.",
                    ["next", "cancel"]) == "cancel":
            return self.cancel(above, grab)
        closed, lift = self.arm.target[5], list(self.arm.target)

        holds = self.ask("check", "Is it holding the cube?", ["yes", "no"]) == "yes"
        self.busy("moving", "Putting the cube back down…")
        self.release_and_home(above, grab)
        if not holds:
            self.event("info", "Not saved.")
            return False
        example = {
            "id": time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:4],
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "kind": kind,
            **{k: reading[k] for k in ("u", "v", "angle", "area_frac", "color", "width", "height", "jitter")},
            "above": above, "grab": grab, "closed": closed, "lift": lift,
        }
        self.session.add_example(example)
        self.session.save_path(example["id"], self.path_log)
        self.examples.append(example)
        self.event("saved", f"Example {len(self.examples)} saved!")
        due = len(self.examples) % TEST_EVERY == 0
        self.refit()
        if due:
            self.save_model()
            self.test(prompt=True)
        return True

    def cancel(self, above, grab=None):
        self.event("info", "Cancelled; opening and going home.")
        self.busy("moving", "Opening and going home…")
        self.release_and_home(above, grab)
        return False

    def test(self, prompt=False):
        if self.model is None:
            self.event("warn", "Record at least one example first.")
            return
        self.go_home()
        if prompt and self.ask("test_ready", f"{len(self.examples)} examples: test time! "
                               "Put ONE cube anywhere and let me try on my own.",
                               ["go", "skip"]) != "go":
            return
        self.busy("reading", "Looking for the cube…")
        reading, why = self.read_cube()
        if reading is None:
            self.event("warn", f"Cannot test: {why}.")
            return
        guess = self.model.predict(reading)
        record = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "model": self.model.info.get("name"),
                  "chosen": self.model.info.get("chosen"), "reading": reading,
                  "guess": {k: guess[k] for k in ("above", "grab", "grip", "nearest_px")}}
        if guess["nearest_px"] > UNSURE_PX:
            self.session.log_test({**record, "result": "refused"})
            if self.ask("teach_here", f"I have not learned this spot yet (closest example "
                        f"{guess['nearest_px']:.0f} px away). Teach it here now?", ["yes", "no"]) == "yes":
                self.record_example(reading)
            return

        self.busy("testing", f"Going for the {reading['color']} cube on my own…")
        try:
            self.move(guess["above"])
            self.move(guess["grab"])
            self.move(guess["grip"], settle=0.8)
            self.move(guess["lift"])
            self.go_home(gripper=ws.GRIPPER_CLOSED)
        except Stopped:
            self.session.log_test({**record, "result": "stopped"})
            raise
        self.busy("reading", "Did I get it?")
        after = self.arm.fresh_frames(FRAMES)
        left = [pl.cube_near(self.detect(f), reading["u"], reading["v"]) for f in after]
        picked = sum(c is None for c in left) > len(left) / 2
        self.session.log_test({**record, "result": "ok" if picked else "fail"})
        self.success = self.success_text()
        if picked:
            self.event("picked", f"Picked it on my own! ({self.success} recent tests)")
            self.busy("moving", "Putting it back…")
            self.move(guess["lift"])
            self.release_and_home(guess["above"], guess["grip"])
            return
        self.event("missed", "Missed: the cube is still on the table.")
        self.go_home()
        if self.ask("correct", "Missed. Leave the cube where it is and show me how?",
                    ["yes", "no"]) == "yes":
            self.record_example(kind="correction")

    def teach_drops(self):
        drops = self.session.drops()
        try:
            for color in list(ws.CUBE_HSV):
                start = drops.get(color) or self.home(ws.GRIPPER_CLOSED)
                self.busy("moving", f"Moving to the {color} drop spot…")
                self.move(start)
                choice = self.ask("drop", f"Drive the closed gripper over where {color.upper()} "
                                  "cubes should go.", ["save", "skip", "cancel"])
                if choice == "save":
                    drops[color] = list(self.arm.target)
                    self.event("saved", f"Saved the {color} drop spot.")
                elif choice == "cancel":
                    break
        finally:
            self.session.save_drops(drops)
        self.go_home()

    def run_review(self):
        self.busy("training", "Checking every example against its neighbours…")
        self.review = pl.review_text(self.examples, self.session.tests())
        suspects = pl.suspect_examples(self.examples)
        try:
            if not suspects or self.ask("review", "Here is what I found.", ["check", "done"]) == "done":
                if not suspects:
                    self.ask("review", "Here is what I found.", ["done"])
                return
            removed = 0
            for s in suspects:
                choice = self.ask("review", "#{} at ({:.0f}, {:.0f}) px: {}. Delete it?".format(
                    s["index"] + 1, s["x"], s["y"], "; ".join(s["reasons"])), ["delete", "keep", "done"])
                if choice == "done":
                    break
                if choice == "delete" and self.session.remove_example(s["id"]):
                    removed += 1
            if removed:
                self.examples = self.session.examples()
                self.refit()
                self.save_model()
                self.event("info", f"Removed {removed} example(s).")
        finally:
            self.review = []

    def undo(self):
        gone = self.session.remove_last_example()
        if not gone:
            self.event("warn", "No examples to undo.")
            return
        self.examples = self.session.examples()
        self.refit()
        self.event("info", f"Removed the last example ({gone['id']}).")
