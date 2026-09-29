"""Run the Edge Impulse cube detector (TFLite) on images or a live webcam.

    python detect.py                    # test1 and test2, shown on screen
    python detect.py --live             # live webcam
    python detect.py photo.jpg --model int8 --conf 0.4

See README.md in this folder for setup.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
SEARCH_DIRS = [HERE, HERE / "models", HERE / "images", REPO]
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp")
LABEL = "cube"
BOX_COLOR = (0, 200, 0)  # BGR


# --- files -------------------------------------------------------------------

def find_model(choice: str) -> Path:
    """``float32``/``int8`` pick a model by name; anything else is a path."""
    path = Path(choice)
    if path.is_file():
        return path
    matches = []
    for folder in SEARCH_DIRS:
        for pattern in ("*.lite", "*.tflite"):
            matches += [p for p in folder.glob(pattern) if choice.lower() in p.name.lower()]
    if not matches:
        sys.exit(
            f"No model matching '{choice}' found in {', '.join(str(d) for d in SEARCH_DIRS)}.\n"
            "Copy the .lite / .tflite files into this folder (or test/models/), or pass "
            "--model /full/path/to/model.lite"
        )
    return matches[0]


def find_image(name: str) -> Path:
    """Accepts ``test1``, ``test1.jpg`` or a path."""
    path = Path(name)
    if path.is_file():
        return path
    for folder in SEARCH_DIRS:
        for candidate in [folder / name] + [folder / (name + s) for s in IMAGE_SUFFIXES]:
            if candidate.is_file():
                return candidate
    sys.exit(f"Image '{name}' not found in {', '.join(str(d) for d in SEARCH_DIRS)}")


# --- model ---------------------------------------------------------------------

def load_interpreter(model_path: Path):
    try:
        from ai_edge_litert.interpreter import Interpreter
    except ImportError:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            try:
                from tensorflow.lite.python.interpreter import Interpreter
            except ImportError:
                sys.exit(
                    "No TFLite runtime installed. Run: pip install -r requirements.txt "
                    "(see README.md; needs Python 3.9-3.12)."
                )
    interpreter = Interpreter(model_path=str(model_path))
    interpreter.allocate_tensors()
    return interpreter


class CubeDetector:
    """Edge Impulse YOLO-style detector: 64x64 RGB in, ``[N, 5]`` out.

    Each output row is ``x1, y1, x2, y2, score`` with coordinates normalised
    to 0-1 (the decoder also accepts pixel coordinates in the model's input
    space). The input is RGB scaled to 0-1, squashed to the model size.
    """

    def __init__(self, model_path: Path, conf: float = 0.3, iou: float = 0.45):
        self.model_path = model_path
        self.conf = conf
        self.iou = iou
        self.interp = load_interpreter(model_path)
        self.inp = self.interp.get_input_details()[0]
        self.out = self.interp.get_output_details()[0]
        _, self.in_h, self.in_w, _ = self.inp["shape"]
        self.last_raw = None

    def _prepare(self, bgr: np.ndarray) -> np.ndarray:
        small = cv2.resize(bgr, (self.in_w, self.in_h), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        if self.inp["dtype"] == np.int8:
            scale, zero = self.inp["quantization"]
            rgb = np.clip(np.round(rgb / scale + zero), -128, 127)
        return rgb.astype(self.inp["dtype"])[None]

    def _raw_output(self) -> np.ndarray:
        out = self.interp.get_tensor(self.out["index"])[0].astype(np.float32)
        if self.out["dtype"] == np.int8:
            scale, zero = self.out["quantization"]
            out = (out - zero) * scale
        if out.shape[0] < out.shape[1]:  # [5, N] -> [N, 5]
            out = out.T
        return out

    def detect(self, bgr: np.ndarray):
        """Returns a list of (x1, y1, x2, y2, score) in pixels of ``bgr``."""
        height, width = bgr.shape[:2]
        self.interp.set_tensor(self.inp["index"], self._prepare(bgr))
        self.interp.invoke()
        raw = self._raw_output()
        self.last_raw = raw

        scores = raw[:, 4]
        keep = scores >= self.conf
        boxes, scores = raw[keep, :4].copy(), scores[keep]
        if len(scores) == 0:
            return []
        # Judge the units from confident rows only: low-score rows hold junk.
        if boxes.max() > 1.5:  # pixel units of the model input
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


# --- drawing -----------------------------------------------------------------------

def draw(bgr: np.ndarray, detections) -> np.ndarray:
    canvas = bgr.copy()
    thickness = max(2, round(max(canvas.shape[:2]) / 400))
    scale = max(0.5, max(canvas.shape[:2]) / 900)
    for x1, y1, x2, y2, score in detections:
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(canvas, p1, p2, BOX_COLOR, thickness)
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        cv2.drawMarker(canvas, (cx, cy), BOX_COLOR, cv2.MARKER_CROSS, 12 * thickness, thickness)
        caption = f"{LABEL} {score:.2f}"
        (tw, th), base = cv2.getTextSize(caption, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
        top = max(0, p1[1] - th - base - 4)
        cv2.rectangle(canvas, (p1[0], top), (p1[0] + tw + 6, top + th + base + 4), BOX_COLOR, -1)
        cv2.putText(canvas, caption, (p1[0] + 3, top + th + 1), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (0, 0, 0), thickness, cv2.LINE_AA)
    return canvas


def fit_to_screen(image: np.ndarray, max_h: int = 900, max_w: int = 1400) -> np.ndarray:
    h, w = image.shape[:2]
    factor = min(1.0, max_h / h, max_w / w)
    if factor >= 1.0:
        return image
    return cv2.resize(image, (int(w * factor), int(h * factor)), interpolation=cv2.INTER_AREA)


def report(name: str, detections, ms: float) -> None:
    print(f"{name}: {len(detections)} cube(s) in {ms:.1f} ms")
    for n, (x1, y1, x2, y2, score) in enumerate(detections, 1):
        print(f"  #{n} score={score:.2f} box=({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f}) "
              f"centre=({(x1 + x2) / 2:.0f},{(y1 + y2) / 2:.0f}) px")


# --- modes -------------------------------------------------------------------------------

def run_images(detector: CubeDetector, names, show: bool, save: bool, debug: bool) -> None:
    out_dir = HERE / "output"
    for name in names:
        path = find_image(name)
        image = cv2.imread(str(path))
        if image is None:
            print(f"Could not read {path}")
            continue
        start = time.perf_counter()
        detections = detector.detect(image)
        ms = (time.perf_counter() - start) * 1000
        report(path.name, detections, ms)
        if debug and detector.last_raw is not None:
            top = detector.last_raw[np.argsort(-detector.last_raw[:, 4])[:5]]
            print("  top raw rows (x1,y1,x2,y2,score):")
            for row in top:
                print("   ", np.round(row, 3).tolist())

        annotated = draw(image, detections)
        if save:
            out_dir.mkdir(exist_ok=True)
            target = out_dir / f"{path.stem}_detected.jpg"
            cv2.imwrite(str(target), annotated)
            print(f"  saved {target}")
        if show:
            cv2.imshow(f"{path.name}  (any key: next, q: quit)", fit_to_screen(annotated))
            key = cv2.waitKey(0) & 0xFF
            cv2.destroyAllWindows()
            if key in (ord("q"), 27):
                break


# Keys that nudge the live confidence threshold, and by how much.
CONF_STEP_KEYS = {
    ord("+"): 0.01, ord("="): 0.01, ord("]"): 0.05,
    ord("-"): -0.01, ord("_"): -0.01, ord("["): -0.05,
}


def run_live(detector: CubeDetector, camera: int) -> None:
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
    cap = cv2.VideoCapture(camera, backend)
    if not cap.isOpened():
        sys.exit(f"Could not open camera {camera}. Try --camera 1.")
    print("Live: q/Esc quits, s saves a snapshot")
    print("      [ / ] adjust the confidence threshold by 0.05, - / + by 0.01")
    print(f"      starting threshold: {detector.conf:.2f}")
    fps, last = 0.0, time.perf_counter()
    snapshots = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera returned no frame")
                break
            start = time.perf_counter()
            detections = detector.detect(frame)
            infer_ms = (time.perf_counter() - start) * 1000
            now = time.perf_counter()
            fps = 0.9 * fps + 0.1 / max(now - last, 1e-6)
            last = now

            annotated = draw(frame, detections)
            cv2.putText(annotated, f"{len(detections)} cube(s)  {infer_ms:.0f} ms  {fps:.0f} fps",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(annotated, f"conf >= {detector.conf:.2f}  ([ ] or - + to adjust)",
                        (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("cube detector (q to quit)", fit_to_screen(annotated))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                (HERE / "output").mkdir(exist_ok=True)
                target = HERE / "output" / f"live_{snapshots}.jpg"
                cv2.imwrite(str(target), annotated)
                print(f"saved {target}")
                snapshots += 1
            elif key in CONF_STEP_KEYS:
                detector.conf = round(min(1.0, max(0.0, detector.conf + CONF_STEP_KEYS[key])), 2)
                print(f"confidence threshold -> {detector.conf:.2f}")
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("images", nargs="*", default=["test1", "test2"],
                        help="image names or paths (default: test1 test2)")
    parser.add_argument("--live", action="store_true", help="use the webcam instead of images")
    parser.add_argument("--camera", type=int, default=0, help="webcam index for --live (default 0)")
    parser.add_argument("--model", default="float32",
                        help="'float32', 'int8', or a path to a .lite/.tflite file (default float32)")
    parser.add_argument("--conf", type=float, default=0.3, help="score threshold (default 0.3)")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU threshold (default 0.45)")
    parser.add_argument("--save", action="store_true", help="also write annotated images to test/output/")
    parser.add_argument("--no-show", action="store_true", help="do not open a window (implies --save)")
    parser.add_argument("--debug", action="store_true", help="print the top raw model outputs")
    args = parser.parse_args()

    model_path = find_model(args.model)
    detector = CubeDetector(model_path, conf=args.conf, iou=args.iou)
    print(f"Model: {model_path.name}  input {detector.in_w}x{detector.in_h}  conf>={args.conf}")

    if args.live:
        run_live(detector, args.camera)
    else:
        run_images(detector, args.images, show=not args.no_show,
                   save=args.save or args.no_show, debug=args.debug)


if __name__ == "__main__":
    main()
