# 11. One launch for everything

## Objective

`real.launch.py` starts the arm, the camera and cube detection together.
`pi.launch.py` is the same with good defaults for a headless Raspberry Pi, plus
the web dashboard.

## Why we need this now

Starting four programs by hand, with the right options, is error-prone and won't
survive a reboot. A launch file *is* the system's wiring diagram: which nodes run,
which topics they use, and with what settings.

## Create

1. In bringup: `launch/real.launch.py`, `launch/pi.launch.py`, and
   `unoq_braccio_bringup/real_camera.py` (copy as-is: it reads the camera
   settings that `real_setup` saved).
2. Copy these driver nodes as-is, because the launch file starts them:
    - `sim_cube_detector.py`, with its helpers `cube_model_backend.py` and
      `table_projection.py`. It publishes the cubes it sees and the camera image
      with labelled boxes; despite its name, it serves the real arm too.
    - `workspace_markers.py`: the table outline in RViz.
3. Add their entry points (copy the lines from the original `setup.py`):
   `sim_cube_detector`, `workspace_markers`, `real_setup`.

## The code

### `real.launch.py`

@code ros2_ws/src/unoq_braccio_bringup/launch/real.launch.py

Read it top to bottom:

1. **Arguments:** every option in one place, each with a default and a description.
2. **`IncludeLaunchDescription(hardware.launch.py)`:** reuse chapter 5 instead of repeating it.
3. **`overhead_camera`:** chapter 6's node with this project's topic.
4. **`cube_detector`:** an `OpaqueFunction`, because whether it starts depends on
   a *file* (the saved camera setup) that is only read at launch time. On the very
   first run there's no file yet, so the arm and camera start without detection
   and you can run `real_setup`.
5. **`web`:** only with `web:=true` (chapter 12).

### `pi.launch.py`

@code ros2_ws/src/unoq_braccio_bringup/launch/pi.launch.py

A **wrapper**: the same system, with the web page always on and RViz off (a Pi
has no screen). Wrappers like this keep the main launch file general.

## Run it

```bash
ros2 launch unoq_braccio_bringup real.launch.py serial_port:=auto camera:=0 rviz:=true
```

Passes when the arm stands up and RViz shows the arm. Then `ros2 node list` shows
the bridge, `robot_state_publisher`, `overhead_camera` and `cube_detector`.

To start it at boot, see step 9 of `docs/raspberry-pi.md` in the original repo
(a `systemd` service).

## What's next

Everything runs, but only from a terminal. Next, anyone on the network can watch
and drive it from a phone: chapter 12.
