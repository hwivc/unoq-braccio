# 5. Describe and launch the arm

## Objective

Give ROS the arm's shape (a URDF), convert servo degrees into URDF joint angles,
start everything with one launch file, and make the arm do a trick from a
second node.

## Why we need this now

`/joint_states` from chapter 4 are just six numbers. To *see* the arm (in RViz now,
and on the web page in chapter 12) ROS needs to know the link lengths, how each
joint turns, and what the parts look like. With that, `robot_state_publisher` turns
joint angles into 3D positions (TF). The same maths later tells us where the
fingertip is.

## Create

1. Copy the meshes as-is: `cp -r $ORIG/ros2_ws/src/unoq_braccio_sim/meshes $NEW/ros2_ws/src/unoq_braccio_sim/`.
   They're GPL-3.0 STL files from Will Stedden's `braccio_moveit_gazebo`.
2. `unoq_braccio_sim/urdf/braccio.urdf.xacro` (below), and copy `rviz/braccio.rviz` as-is.
3. In the driver package: `braccio_kinematics.py` and `arm_tricks.py`.
4. In bringup: `launch/hardware.launch.py`.

## The code

### The URDF: links and joints

A URDF is a tree. **Links** are the rigid parts; **joints** connect a parent link to
a child link, at an offset (`origin`), turning about an `axis`.

@code ros2_ws/src/unoq_braccio_sim/urdf/braccio.urdf.xacro L34-L76

@code ros2_ws/src/unoq_braccio_sim/urdf/braccio.urdf.xacro L142-L168

Type the remaining links and joints the same way (lines 77–218). The `ros2_control`
and `gazebo` blocks after line 219 are for the simulation only, so copy them as-is.

### Servo degrees ↔ URDF radians

The UNO speaks servo degrees (90 = centre); the URDF speaks radians, about its
own zero. These are the constants measured from the URDF:

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_kinematics.py L1-L51

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_kinematics.py servo_to_urdf

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_kinematics.py servo_positions_to_urdf

This is what `serial_bridge` calls before publishing `/joint_states`.

### Forward kinematics: where is the fingertip?

Small 3×3 rotation helpers, then the URDF chain copied as numbers:

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_kinematics.py L76-L120

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_kinematics.py _tool_pose

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_kinematics.py forward_kinematics

Walk the chain from the base: move by the joint's offset (rotated into the current
frame), then turn by the joint's angle. After five joints you're at the wrist;
add the finger length and you have the fingertip.

??? note "Optional: inverse kinematics (used by the simulation)"
    The real arm doesn't need IK: it learns picks (chapter 8). The simulation's
    pick-and-place does use it, and it's good maths to know.

    @code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_kinematics.py _two_link

    @code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_kinematics.py solve_ik

    The **law of cosines** gives the elbow bend from the distance to the wrist;
    `atan2` gives the shoulder angle. Try tool pitches from straight down towards
    level, and take the first answer inside the servo limits.

### One launch file for the hardware

@code ros2_ws/src/unoq_braccio_bringup/launch/hardware.launch.py

Three nodes:

- the bridge (chapter 4),
- `robot_state_publisher`, fed the URDF by running `xacro` at launch,
- RViz, only if `rviz:=true`.

`DeclareLaunchArgument` + `LaunchConfiguration` is how a launch file takes options.

### Your first client: arm tricks

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/arm_tricks.py

A trick is a list of `(pose, seconds)`. The node only **publishes `JointState`**;
the bridge does the rest. Every later node moves the arm exactly like this.
Add the entry point: `"arm_tricks = unoq_braccio_driver.arm_tricks:main",`.

## Run it

```bash
cd $NEW/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch unoq_braccio_bringup hardware.launch.py serial_port:=auto rviz:=true
# second terminal:
ros2 run unoq_braccio_driver arm_tricks --ros-args -p trick:=wave
```

Passes when the real arm waves and RViz's arm waves with it. On a PC, check the
maths without ROS: copy `test/test_workspace.py` from the original and run it.

## What's next

The arm moves and we can see it, but it's blind. To sort cubes it needs a
picture of the table: chapter 6.
