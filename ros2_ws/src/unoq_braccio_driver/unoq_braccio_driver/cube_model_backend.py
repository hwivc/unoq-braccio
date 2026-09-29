"""Pluggable cube-box detectors for the overhead camera.

``sim_cube_detector`` needs one thing from this module: a box around each
cube in an image. Where those boxes come from is deliberately swappable -
today it is an Edge Impulse TFLite model, and a later, better, or
per-colour-class model drops in the same way - through the ``CubeBoxDetector``
interface and :func:`create_cube_detector`. Colour is decided separately, by
sampling pixels inside each box (see ``color_vision.best_color``), because a
single-class "cube" model does not report it.

Swap in a different model with no code changes to ``sim_cube_detector.py``:

    ros2 launch unoq_braccio_bringup sim.launch.py \\
        model_path:=/path/to/your_model.lite

or point ``EDGE_IMPULSE_CUBE_MODEL`` at it. A model with a different input
size or class count works unchanged; only a model whose output is not
``[N, 5]`` rows of ``x1, y1, x2, y2, score`` would need a new backend class.
"""

from __future__ import annotations

import glob
import os
from pathlib import Path

import numpy as np


class CubeBoxDetector:
    """Common interface: an RGB image in, cube boxes out. No colour, no ROS."""

    name = "base"

    def find_cubes(self, rgb: np.ndarray):
        """Returns a list of ``(x1, y1, x2, y2, score)`` in pixels of ``rgb``."""
        raise NotImplementedError


class ColorBlobCubeDetector(CubeBoxDetector):
    """Fallback with no model: any blob in a cube colour range is "a cube".

    Used automatically when no Edge Impulse model can be loaded, and useful
    as a baseline to compare a trained model against (``detector_backend:=
    color_blob``). Every box gets a fixed score of 1.0, since there is no
    learned confidence to report.
    """

    name = "color_blob"

    def __init__(self, cube_size_m: float, camera_fx: float, camera_height_m: float):
        from unoq_braccio_driver import braccio_workspace as ws

        self._ranges = ws.CUBE_HSV
        cube_px = cube_size_m * camera_fx / camera_height_m
        self._min_area, self._max_area = 0.4 * cube_px ** 2, 2.5 * cube_px ** 2

    def find_cubes(self, rgb: np.ndarray):
        import cv2

        from unoq_braccio_driver.color_vision import to_hsv

        hsv = to_hsv(rgb)
        boxes = []
        for ranges in self._ranges.values():
            mask = None
            for low, high in ranges:
                part = cv2.inRange(hsv, np.array(low), np.array(high))
                mask = part if mask is None else cv2.bitwise_or(mask, part)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if not self._min_area <= cv2.contourArea(contour) <= self._max_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                boxes.append((float(x), float(y), float(x + w), float(y + h), 1.0))
        return boxes


class EdgeImpulseCubeDetector(CubeBoxDetector):
    """Boxes from an Edge Impulse TFLite model.

    Decoding matches ``test/detect.py``: whatever input size the model
    declares, RGB scaled to 0-1 (quantised for an int8 model), giving
    ``[N, 5]`` rows of ``x1, y1, x2, y2, score`` normalised to 0-1.
    """

    name = "edge_impulse"

    def __init__(self, model_path: str, conf: float = 0.3, iou: float = 0.45):
        self.model_path = model_path
        self.conf = conf
        self.iou = iou
        self._interp = _load_interpreter(model_path)
        self._input = self._interp.get_input_details()[0]
        self._output = self._interp.get_output_details()[0]
        _, self.in_h, self.in_w, _ = self._input["shape"]

    def find_cubes(self, rgb: np.ndarray):
        import cv2

        height, width = rgb.shape[:2]
        small = cv2.resize(rgb, (self.in_w, self.in_h), interpolation=cv2.INTER_AREA)
        x = small.astype(np.float32) / 255.0
        if self._input["dtype"] == np.int8:
            scale, zero = self._input["quantization"]
            x = np.clip(np.round(x / scale + zero), -128, 127)
        self._interp.set_tensor(self._input["index"], x.astype(self._input["dtype"])[None])
        self._interp.invoke()

        raw = self._interp.get_tensor(self._output["index"])[0].astype(np.float32)
        if self._output["dtype"] == np.int8:
            scale, zero = self._output["quantization"]
            raw = (raw - zero) * scale
        if raw.shape[0] < raw.shape[1]:  # [5, N] -> [N, 5]
            raw = raw.T

        scores = raw[:, 4]
        keep = scores >= self.conf
        boxes, scores = raw[keep, :4].copy(), scores[keep]
        if len(scores) == 0:
            return []
        # Judge the units from confident rows only: low-score rows hold junk.
        if boxes.max() > 1.5:  # pixel units of the model's own input size
            boxes[:, [0, 2]] /= self.in_w
            boxes[:, [1, 3]] /= self.in_h
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0.0, 1.0) * width
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0.0, 1.0) * height

        wh = boxes[:, 2:4] - boxes[:, 0:2]
        valid = (wh > 1).all(axis=1)
        boxes, scores = boxes[valid], scores[valid]
        if len(scores) == 0:
            return []

        xywh = [[float(b[0]), float(b[1]), float(b[2] - b[0]), float(b[3] - b[1])] for b in boxes]
        picked = cv2.dnn.NMSBoxes(xywh, [float(s) for s in scores], self.conf, self.iou)
        return [(*(float(v) for v in boxes[i]), float(scores[i])) for i in np.array(picked).flatten()]


def _load_interpreter(model_path: str):
    try:
        from ai_edge_litert.interpreter import Interpreter
    except ImportError:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            from tensorflow.lite.python.interpreter import Interpreter  # last resort
    interp = Interpreter(model_path=model_path)
    interp.allocate_tensors()
    return interp


def _search_roots():
    """Directories to look for a ``.lite``/``.tflite`` model in, most useful first."""
    roots = [os.getcwd()]
    here = Path(__file__).resolve()  # follows the colcon --symlink-install symlink
    roots += [str(p) for p in here.parents]
    roots.append(os.path.expanduser("~/unoq-braccio"))
    seen = set()
    return [r for r in roots if not (r in seen or seen.add(r))]


def find_model_file(explicit_path: str = ""):
    """Resolve a model path: an explicit path, then ``EDGE_IMPULSE_CUBE_MODEL``,
    then a search of the working directory, the repository, and the home
    directory, preferring a float32 model (more accurate) over any other."""
    if explicit_path:
        return explicit_path if os.path.isfile(explicit_path) else None
    env = os.environ.get("EDGE_IMPULSE_CUBE_MODEL", "")
    if env and os.path.isfile(env):
        return env
    roots = _search_roots()
    for pattern in ("*float32*.lite", "*float32*.tflite", "*.lite", "*.tflite"):
        for base in roots:
            matches = sorted(glob.glob(os.path.join(base, pattern)))
            if matches:
                return matches[0]
    return None


def create_cube_detector(
    backend: str,
    model_path: str,
    conf: float,
    iou: float,
    cube_size_m: float,
    camera_fx: float,
    camera_height_m: float,
    logger=None,
) -> CubeBoxDetector:
    """Builds the requested backend, falling back to colour blobs on failure.

    ``backend`` is ``"edge_impulse"`` (default) or ``"color_blob"``. Any other
    value, a missing model file, or a backend that fails to load (for example
    no TFLite runtime installed) falls back to colour blobs with a warning, so
    the simulation still runs rather than the node exiting.
    """

    def log(message: str, warn: bool = False) -> None:
        if logger is not None:
            (logger.warning if warn else logger.info)(message)
        else:
            print(message)

    if backend == "edge_impulse":
        resolved = find_model_file(model_path)
        if resolved is None:
            log(
                "No Edge Impulse model found (set the 'model_path' parameter, or "
                "EDGE_IMPULSE_CUBE_MODEL, or place a .lite file in the repository "
                "root). Falling back to colour-blob detection.",
                warn=True,
            )
        else:
            try:
                detector = EdgeImpulseCubeDetector(resolved, conf=conf, iou=iou)
                log(
                    f"Cube detector: Edge Impulse model '{resolved}' "
                    f"({detector.in_w}x{detector.in_h}, conf>={conf})"
                )
                return detector
            except Exception as exc:  # missing runtime, corrupt file, wrong shape, ...
                log(
                    f"Could not load Edge Impulse model '{resolved}': {exc}. "
                    "Falling back to colour-blob detection.",
                    warn=True,
                )
    elif backend != "color_blob":
        log(f"Unknown detector_backend '{backend}'; using colour-blob detection.", warn=True)

    detector = ColorBlobCubeDetector(cube_size_m, camera_fx, camera_height_m)
    log("Cube detector: colour-blob (no model)")
    return detector
