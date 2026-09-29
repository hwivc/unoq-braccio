# Cube detector test (Edge Impulse TFLite)

Runs the Edge Impulse cube model on still images or a live webcam and shows the
detections on screen. It is a stand-alone check of the model; it does not use
ROS.

## What is in this folder

| File | Purpose |
| --- | --- |
| `detect.py` | The test script |
| `requirements.txt` | Python packages (`ai-edge-litert`, `numpy`, `opencv-python`) |
| `output/` | Annotated images written by `--save` / `--no-show` (git-ignored) |

The script looks for these files in `test/`, `test/models/`, `test/images/`, and
the repository root, in that order:

- the model files: `*float32*.lite` and `*int8*.lite`
- the test images: `test1.jpg` and `test2.jpg`

So leaving them in the repository root works, and so does moving them here.

## 1. Set up Python

Use **Python 3.9 to 3.12** (tested on 3.11). Very new Python versions
(for example 3.14) may not have a TFLite runtime wheel yet.

### Windows (PowerShell)

```powershell
cd test
py -3.11 -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
```

If PowerShell blocks activation, run this once in the same window and try again:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
```

### Linux / macOS

```bash
cd test
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

On a Raspberry Pi or other ARM Linux, if `ai-edge-litert` has no wheel for your
Python, install `tflite-runtime` instead (`pip install tflite-runtime numpy
opencv-python`); the script picks up whichever is installed.

On a headless machine install `opencv-python-headless` instead of
`opencv-python` and use `--no-show`.

## 2. Run it

With the virtual environment active, from the `test/` folder:

```bash
python detect.py                  # test1 and test2, shown one after the other
python detect.py --live           # live webcam
```

In the image window press any key for the next image, `q` or `Esc` to quit. In
live mode `q` or `Esc` quits and `s` saves a snapshot to `output/`.

### Live threshold tuning

In `--live` mode the confidence threshold (`--conf`) can be changed while the
camera is running, without restarting:

| Key | Effect |
| --- | --- |
| `]` | Raise the threshold by 0.05 (fewer, more confident boxes) |
| `[` | Lower the threshold by 0.05 (more boxes, more false positives) |
| `+` / `=` | Raise the threshold by 0.01 (fine adjustment) |
| `-` / `_` | Lower the threshold by 0.01 (fine adjustment) |

The current value is drawn on the video (`conf >= 0.30`) and printed to the
terminal each time it changes. Start from `--conf` on the command line and
nudge it up while watching the feed until bad detections (empty background,
shadows, the arm itself) drop out but the real cube still shows. That value is
the one to use for `--conf` next time, and later for whatever score threshold
the pick-and-place pipeline applies to `/edge_impulse/label` or its detections
topic.

Options:

| Option | Meaning |
| --- | --- |
| `--live` | Use the webcam instead of images |
| `--camera N` | Webcam index for `--live` (default 0; try 1 for a second camera) |
| `--model float32` / `int8` / `PATH` | Which model (default `float32`, the more accurate one) |
| `--conf 0.3` | Minimum score to show a box (default 0.3) |
| `--iou 0.45` | Overlap threshold for merging duplicate boxes |
| `--save` | Also write annotated images to `output/` |
| `--no-show` | Do not open a window; write to `output/` (for SSH or headless use) |
| `--debug` | Print the highest-scoring raw model outputs |
| `IMAGE ...` | Your own images, by name or path: `python detect.py photo.jpg` |

Examples:

```bash
python detect.py --model int8                # quantized model, about 2x faster
python detect.py test1 --conf 0.15 --debug   # one image, show weaker detections
python detect.py --live --camera 1 --conf 0.4
```

The console prints each detection with its score, box and centre in pixels of
the original image, plus the inference time.

## About the model

The model was trained in Edge Impulse (project
<https://studio.edgeimpulse.com/public/975321/live>) with one class, `cube`.

- **Input:** 64x64 RGB image, values 0 to 1 (the int8 model takes the same
  values, quantized). The script squashes the whole image to 64x64 with no
  cropping, which is how Edge Impulse resizes by default.
- **Output:** 84 candidate rows of `x1, y1, x2, y2, score`, with the box
  corners normalised to 0 to 1. The script drops rows below `--conf` and merges
  overlapping duplicates.
- **Colour order matters:** the model expects RGB. Feeding it BGR (OpenCV's
  default) noticeably lowers the scores, and the script converts for you.

## Reading the results

- A score is the model's confidence, not a probability. With a 64x64 input the
  scores are naturally modest; a real cube usually lands between 0.4 and 0.8.
- **Low scores or misses on a cube colour** mean the training data did not
  contain enough of that colour, background or camera angle. Compare the same
  image with `--conf 0.1`: if the cube only shows up at a very low score, add
  more labelled examples of it in Edge Impulse and retrain.
- **Loose boxes:** at 64x64 a cube covers only a few pixels, so box edges are
  coarse. Use a tighter camera framing (cube filling more of the image) for
  better boxes.
- **float32 vs int8:** int8 runs about twice as fast and gave nearly the same
  scores on the two test images. Use float32 to judge accuracy and int8 for the
  Grove Vision AI / edge deployment.

## Troubleshooting

- `No TFLite runtime installed`: activate the virtual environment and run
  `pip install -r requirements.txt`.
- `No model matching 'float32' found`: put the `.lite` files in `test/` or the
  repository root, or pass `--model C:\path\to\model.lite`.
- `Could not open camera 0`: close other apps using the webcam, or try
  `--camera 1`.
- `cv2.imshow` errors about GUI support: you installed the headless OpenCV.
  Reinstall `opencv-python`, or use `--no-show`.
- The model files are about 155 MB (float32) and 40 MB (int8). GitHub rejects
  files over 100 MB, so `*.lite` and `*.tflite` are git-ignored. Keep the models
  local, or use Git LFS if you want them in the repository.
