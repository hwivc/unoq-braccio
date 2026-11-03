# 1. Workspace and packages

## Objective

Create an empty ROS 2 workspace with the three packages the project is split into.

## Why we need this first

ROS 2 finds code by **package**. Every node we write later must live in one, and
`colcon build` must know about it before `ros2 run` can start it. We split the
project by job:

| Package | Type | Holds |
|---|---|---|
| `unoq_braccio_driver` | Python (`ament_python`) | every node: serial bridge, camera, vision, learning, dashboard |
| `unoq_braccio_bringup` | Python | launch files and config: *which* nodes to start, *with what settings* |
| `unoq_braccio_sim` | CMake (`ament_cmake`) | the arm's description (URDF), meshes and RViz config, which are data, not code |

Keeping launch files apart from nodes means you can change how things start
without touching the code that runs.

## Create

```bash
mkdir -p $NEW/ros2_ws/src && cd $NEW && git init
cd ros2_ws/src
ros2 pkg create --build-type ament_python unoq_braccio_driver \
    --dependencies rclpy sensor_msgs std_msgs
ros2 pkg create --build-type ament_python unoq_braccio_bringup
ros2 pkg create --build-type ament_cmake unoq_braccio_sim
rm -rf unoq_braccio_sim/include unoq_braccio_sim/src     # the sim package holds no C++
mkdir -p $NEW/firmware/braccio_uno_firmware
```

## The code

### The driver package's `setup.py`

`ros2 pkg create` wrote a `setup.py`. Make yours match this beginning:

@code ros2_ws/src/unoq_braccio_driver/setup.py L1-L20

- `package_data` ships the web page's files with the package (chapter 12).
- `install_requires` lists the pip packages the nodes import.

Below that comes `entry_points`. Every line there becomes a command:
`"serial_bridge = unoq_braccio_driver.serial_bridge:main"` means
`ros2 run unoq_braccio_driver serial_bridge` calls `main()` in `serial_bridge.py`.
**Start with an empty list** and add one line each time a chapter creates a node:

```python
    entry_points={
        "console_scripts": [
            # added chapter by chapter
        ],
    },
)
```

### The driver package's `package.xml`

@code ros2_ws/src/unoq_braccio_driver/package.xml

`<depend>` lines are ROS packages we import (`rclpy`, message types).
`<exec_depend>` lines are system packages needed at run time (OpenCV, pyserial,
YAML). `rosdep install` reads these to install everything.

### The bringup package's `setup.py`

@code ros2_ws/src/unoq_braccio_bringup/setup.py

The `data_files` globs copy every `launch/*.launch.py` and `config/*.yaml` into
the install folder, which is where `ros2 launch` looks for them. Create the folders:
`mkdir unoq_braccio_bringup/launch unoq_braccio_bringup/config`.

### The sim package's `CMakeLists.txt`

@code ros2_ws/src/unoq_braccio_sim/CMakeLists.txt

There's nothing to compile, just folders to install. Create them now and fill them in chapter 5:
`mkdir -p unoq_braccio_sim/{config,launch,meshes,rviz,urdf,worlds}`.

## Run it

```bash
cd $NEW/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep unoq_braccio
```

Passes when all three packages are listed. `--symlink-install` links your Python
files instead of copying them, so edits work without rebuilding (except when you
add a new entry point).

## What's next

We have packages but no way to move anything. The arm's servos are wired to an
Arduino UNO, so the first real code runs **on the UNO**: chapter 2.
