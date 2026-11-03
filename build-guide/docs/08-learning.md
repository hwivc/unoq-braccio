# 8. Learn picks from examples

## Objective

Given a cube reading `(u, v, angle)`, predict the servo angles that pick it. The
prediction is learned from examples the arm really did, with no camera calibration
and no inverse kinematics.

## Why we need this now

We could compute picks with IK (chapter 5), but that needs the camera's exact
position and the arm's exact geometry. On a real Braccio the servos are offset,
the links flex and the camera is tilted, so calibrated IK missed often. Instead:

> **Store what the arm did when it really picked a cube. For a new cube, blend
> the stored examples nearest to it.**

Whatever was wrong with the arm or the camera is already baked into the examples.
That's the idea of this project, and this chapter is its heart. It's all plain
numpy, so you can test it on any PC.

## Create

Continue `pick_learning.py` below the camera section from chapter 7, in this order.

## The code

### The regressors: three ways to map inputs to outputs

All three take standardised inputs and outputs (chapter-level detail: subtract the
mean, divide by the spread) and share the same small interface: `fit`, `predict`,
`clone`, `to_dict` and `from_dict`.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py Knn

**kNN:** take the 4 nearest examples, and weight each by `1 / distance²`. A cube
sitting right on a taught spot repeats that example almost exactly.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py _poly_features

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py Poly

**Polynomial:** a smooth surface through all the examples (ridge regression: the
`λI` term stops it overfitting).

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py Mlp

**Tiny neural network:** one hidden layer (`tanh`), trained with Adam, written by
hand in numpy. It's worth reading slowly: forward pass, error, gradients, update.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py L281-L291

### Training data: inputs, targets and the wrist roll

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py _inputs

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py _roll_targets

The wrist must turn with the cube. Rather than learn the roll directly, we learn
`roll = sign × cube_angle + offset(u, v)`, trying both signs (the camera may be
mirrored compared with the wrist) and keeping the steadier one.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py _targets

### Choosing the best model fairly

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py cross_validate

**Cross-validation:** hide some examples, train on the rest, measure the error on
the hidden ones. A model is never scored on data it has seen.

### The pick model

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py PickModel

- `fit` standardises, fits the chosen regressor, and remembers the gripper's open and closed values.
- `predict` returns four poses: `above`, `grab`, `grip` (grab with the gripper
  closed) and `lift`. It also returns `nearest_px`: how far this cube is from any
  example. Later chapters **refuse** cubes that are too far, rather than guess.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py train

Every refit runs the race: kNN, three polynomials and the MLP. With ~47 examples
kNN wins (about 1.7° error), but the others can overtake it as examples grow.

### Checking the examples

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py L439-L442

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py suspect_examples

One badly taught example drags nearby picks off. Compare each example with what
its neighbours predict, flag the worst, remove it, and check again.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py misses

Type `review_text` from the original too; it prints these results for the terminal.

### Saving everything: sessions

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py Session

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py _read_jsonl

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py _append_jsonl

A **session** is one camera and table setup. Examples are JSON Lines, so adding one
never rewrites the file. Every model is kept (`model_0010.json`…) plus `latest.json`.

### Has the camera moved?

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py compare_to_reference

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_learning.py camera_moved

Examples are in pixels, so if the camera moves, they're wrong. This compares the
current view with the session's reference picture (ORB features + RANSAC) and
warns you.

Finish with `main` (L724–L735), a small command-line review tool.

## Run it

```bash
python $NEW/ros2_ws/src/unoq_braccio_driver/test/test_pick_learning.py
```

Passes when every test passes. It trains on synthetic examples and checks the predictions.

## What's next

The learning works, but it needs **real** examples: the arm driven onto real
cubes, with the camera reading recorded. Next is the tool that records them: chapter 9.
