# 10. Sort the cubes

## Objective

A node, `learned_pick_demo`, that clears the table. It looks, chooses the cube it
knows best, picks it, checks with the camera that it's gone, drops it, and
repeats until nothing it can pick is left.

## Why we need this now

This is the project's actual job. Everything so far (moving, seeing, learning,
teaching) exists to make this loop work.

## Create

1. Copy `pick_place_demo.py` as-is. We only use its `State` list, the names
   published on `/task/state` that the web page shows:

    @code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/pick_place_demo.py State

2. `learned_pick_demo.py` in the driver package, and
   `launch/learned_pick_place.launch.py` in bringup.
3. Add the entry point: `"learned_pick_demo = unoq_braccio_driver.learned_pick_demo:main",`

## The code

### Set-up

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/learned_pick_demo.py L1-L58

It inherits `ArmLink` (chapter 9), so `move`, `fresh_frames` and `read_cubes` are already there.

### One pick, checked by the camera

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/learned_pick_demo.py LearnedPickDemo.pick

The model's four poses in order. Success isn't "the gripper closed": it's
**"the camera no longer sees the cube where it was"**, in most of several fresh frames.

### Dropping

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/learned_pick_demo.py LearnedPickDemo.drop_pose

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/learned_pick_demo.py LearnedPickDemo.drop

Turn while standing up, *then* reach out, so the arm never sweeps across the table.

### The loop

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/learned_pick_demo.py LearnedPickDemo.run

Each round, a cube is a candidate only if:

- it was on the table **at the start** (dropped cubes are never re-picked),
- it has a wanted colour,
- the model knows the spot (`nearest_px ≤ unsure_px`),
- there's a drop pose for its colour,
- it hasn't failed too often.

Then pick the **best-known** one (smallest `nearest_px`) first.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/learned_pick_demo.py LearnedPickDemo.go_home

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/learned_pick_demo.py main

### Its launch file

@code ros2_ws/src/unoq_braccio_bringup/launch/learned_pick_place.launch.py

## Run it

With the arm and camera running (as in chapter 9):

```bash
ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=desk
ros2 topic echo /task/state          # watch the steps
```

Passes when the taught cubes are picked and dropped, and the last state is `COMPLETE`.

## What's next

It works, but you start four terminals every time. Next, one launch file starts
everything with good defaults: chapter 11.
