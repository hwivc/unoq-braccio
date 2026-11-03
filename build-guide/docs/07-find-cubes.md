# 7. Find the cubes

## Objective

From one camera frame, list every cube with its colour, centre `(u, v)` and angle.
Then combine several frames into one steady reading.

## Why we need this now

The arm will learn *"when the cube is here in the picture, the servos go there"*
(chapter 8). That only works if "here" is measured the same way every time. This
chapter defines that measurement.

Two choices make it robust:

- Positions are **fractions of the image width** (`u = x / W`, `v = y / W`), so
  changing the camera resolution doesn't break anything learned.
- Colour is **never** a model input, so adding a new cube colour needs no
  retraining, just its colour range.

## Create

1. `color_vision.py` in the driver package.
2. Start `pick_learning.py` with its header and the camera-reading section; chapter 8 adds the rest.
3. Copy as-is (support code): `real_setup.py` and `braccio_workspace.py`. They
   measure and save each cube colour's HSV range to `~/.ros/braccio_setup.yaml`.
   We use their output, not their insides. Add the entry point
   `"real_setup = unoq_braccio_driver.real_setup:main",`.

## The code

### Colour helpers

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/color_vision.py

- `to_hsv`: HSV separates *colour* (hue) from *brightness*, so a red cube stays red in shadow.
- `find_blobs` and `best_color` are used by the simulation's detector; the real arm uses `detect_cubes` below.

### `pick_learning.py`, part 1: reading the camera

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py L1-L35

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py wrap90

A cube looks identical every 90°, so angles are kept in −45..45.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py color_mask

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py detect_cubes

For each colour: mask → clean up specks (`MORPH_OPEN`) → outer contours → reject
anything that isn't cube-like:

| Test | Rejects |
|---|---|
| area too small or too big | noise, a cube held too close |
| touches the edge | a half-visible cube, whose centre would be wrong |
| fills < 60% of its rotated rectangle | the arm, shadows, odd shapes |

The rejected blobs are kept with a reason. The web page draws them as dashed
boxes, so you can *see* why a cube isn't detected.

### Steady readings from several frames

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py summarise

Use the **median** position (one bad frame can't drag it), the most common colour,
and a *circular* mean of the angle, done on `4 × angle` because of the 90°
symmetry. `jitter` records how much the reading wobbled, and chapter 8 flags
wobbly examples with it.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py steady_cubes

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py cube_near

## Run it

Save colours once on the Pi: put one cube under the camera and run
`ros2 run unoq_braccio_driver real_setup --ros-args -p steps:=colors`.

On any PC, check the detection with the original test (it builds synthetic images,
so no camera is needed). The model tests in it will pass after chapter 8:

```bash
cp $ORIG/ros2_ws/src/unoq_braccio_driver/test/test_pick_learning.py $NEW/ros2_ws/src/unoq_braccio_driver/test/
python $NEW/ros2_ws/src/unoq_braccio_driver/test/test_pick_learning.py
```

## What's next

We know where a cube is in the picture. We still don't know which servo angles
reach it, so next the arm *learns* that from examples: chapter 8.
