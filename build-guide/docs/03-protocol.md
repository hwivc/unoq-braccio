# 3. Speak the protocol from Python

## Objective

One place in Python that knows the joints, their limits and named poses, plus the
two functions that turn angles into an `M` line and a `STAT` line back into numbers.

## Why we need this now

Every node from now on will move the arm. If each one built its own `"M ..."`
string, a mistake in one would send the arm somewhere unexpected. Putting the
joint facts and the protocol in two tiny modules means they're written and
tested **once**. They also have no ROS imports, so you can test them on any PC.

## Create

In `$NEW/ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/`:
`braccio_model.py` and `braccio_protocol.py`.

## The code

### `braccio_model.py`: facts about the arm

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_model.py L1-L44

- `JOINT_NAMES` is **the order** of everything: commands, status and poses.
- `JOINT_LIMITS` must match `MIN_LIMITS` and `MAX_LIMITS` in the firmware.
- `START_POSE` is the power-on pose, so the Pi knows where the arm is before the UNO says anything.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_model.py clamp_degrees

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_model.py command_line_from_positions

The input is *names and values* (like a ROS `JointState`), in any order and
possibly missing joints. The output always has all six in `JOINT_NAMES` order, so
the firmware never gets a short or shuffled line.

### `braccio_protocol.py`: the status side

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_protocol.py

`parse_status` is deliberately strict: a wrong number of joints, or a field that
isn't a number, returns `None`, so a garbled line is ignored rather than moving
the 3D view or confusing the code that waits for moves to finish.

## Run it

Copy the test from the original and run it. No ROS is needed:

```bash
mkdir -p $NEW/ros2_ws/src/unoq_braccio_driver/test
cp $ORIG/ros2_ws/src/unoq_braccio_driver/test/test_protocol.py $NEW/ros2_ws/src/unoq_braccio_driver/test/
python $NEW/ros2_ws/src/unoq_braccio_driver/test/test_protocol.py
```

Passes when every line says `PASS`. It also checks that your Python limits match the firmware's.

## What's next

Python can *say* the right words, but something has to own the USB port, keep the
connection alive and connect it to ROS topics: chapter 4.
