# 9. Teach the arm

## Objective

A terminal tool, `teach_pick`, that records real examples. You put a cube down,
the camera reads it, you drive the arm onto it with the keyboard, and it saves
*reading + servo angles*. It refits the model after every example and lets you
test it.

## Why we need this now

Chapter 8's model is empty until it has examples. The only honest source of
examples is the real arm picking real cubes. We also need one reusable piece for
every node that moves the arm and watches the camera, and that piece is `ArmLink`.

## Create

`teach_pick.py` in the driver package. Add
`"teach_pick = unoq_braccio_driver.teach_pick:main",` to the entry points.

## The code

### Keys

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/teach_pick.py L39-L53

### `ArmLink`: the arm and the camera, as one Python object

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/teach_pick.py ArmLink

This is the most reused class in the project. Chapters 10 and 13 inherit from it.

- `on_status` and `on_image` keep the latest arm position and camera frame.
- `move(pose)` publishes a command and **waits** until the reported position is
  within 2° of it (or a timeout based on the distance and speed), so code reads
  like a recipe: `move(above)`, `move(grab)`…
  A `stop()` check lets the user interrupt (`Stopped` is raised).
- `fresh_frames(n)` waits for *new* frames (not old ones from before the move);
  `read_cube` and `read_cubes` call `detect_cubes` and `summarise` or `steady_cubes` from chapter 7.

### Teaching

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/teach_pick.py TeachPick.__init__

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/teach_pick.py TeachPick.refit

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/teach_pick.py TeachPick.drive

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/teach_pick.py TeachPick.record_example

The example's life: **PLACE → ABOVE → GRAB → CLOSE → CHECK.**

- The camera reads the cube *before* the arm moves, so the arm never hides it.
- From the second example on, the arm starts at the **model's own guess**, so
  you only correct it. That makes teaching faster every time.
- An example is saved only if you confirm the cube is held.

### Testing what it learned

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/teach_pick.py TeachPick.test

The arm picks on its own. If it misses, correcting it right there adds an example
exactly where the model was weakest.

### The menu and start-up

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/teach_pick.py TeachPick.menu

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/teach_pick.py spin_in_background

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/teach_pick.py main

ROS callbacks run in a background thread (`spin_in_background`), so the main
thread can block on the keyboard while positions and frames keep arriving.

Type the remaining `TeachPick` methods from the original:
`save_model`, `model_text`, `success_rate`, `yes_no`, `wait_enter`, `go_home`,
`release_and_home`, `cancel`, `offer_correction`, `review`, `teach_drops`,
`check_camera`, `run`.

## Run it

You need the arm and the camera running together. Chapter 11 makes that one
command; for now use two:

```bash
ros2 launch unoq_braccio_bringup hardware.launch.py serial_port:=auto
ros2 run unoq_braccio_driver camera_node --ros-args -p camera:=0
ros2 run unoq_braccio_driver teach_pick --ros-args -p session:=desk
```

Teach 5 examples spread over the table, then press `T` to test.
Passes when the test picks a cube without your help.

## What's next

The arm can pick *one* cube you point at. Next it finds all the cubes, picks
them one by one, checks each pick, and drops them: chapter 10.
