# Roadmap

Planned work that is not built yet: a step-by-step tutorial, and new
simulation features. Nothing here is implemented; it is the plan to work from.

- [1. Step-by-step tutorial](#1-step-by-step-tutorial)
- [2. New simulation features](#2-new-simulation-features)

---

## 1. Step-by-step tutorial

A tutorial that starts from empty packages and builds the whole project in
small steps. Each step has one objective, a little code, the math it needs,
and a result you can see after `colcon build`.

### Branches

| Branch / tag | Contents |
|---|---|
| `main` | The finished project, plus the tutorial text in `tutorial/` |
| `tutorial/start` | Skeleton: empty packages that already build |
| `tutorial/ch01-done` ... `tutorial/ch15-done` | Checkpoint after each chapter, so a learner who is stuck can `git checkout tutorial/ch07-done` and carry on |

Pin the tutorial to a release tag of `main` (e.g. `v1.0`), not to `main`
itself. Otherwise every later change to `main` can break chapters people are
halfway through.

### What `tutorial/start` contains

- `unoq_braccio_driver`, `unoq_braccio_sim` and `unoq_braccio_bringup`, each
  with a minimal `package.xml`, `setup.py` / `CMakeLists.txt` and an empty
  `console_scripts` list. `colcon build` succeeds with no code.
- Assets a learner should not have to make: the STL meshes (with their GPL
  licence) and a download link for the Edge Impulse model.
- **No** nodes, URDF, world, launch files or controller config. The learner
  writes all of these. Each chapter edits `package.xml`, `setup.py`, launch
  files and `controllers.yaml` only when it needs to.

### Chapter format

Every chapter has the same sections:

1. **Objective**: one sentence.
2. **Concepts and math**: only what this chapter needs.
3. **Code**: in small pieces, each followed by an explanation.
4. **Build and run**: exact commands.
5. **Expected result**: a screenshot or GIF.
6. **Check yourself**: a small pure-Python test, like `test/test_workspace.py`.
7. **Checkpoint tag**.

### Chapters

| # | Objective | Math / concepts | Result |
|---|---|---|---|
| 0 | Install ROS 2 Jazzy and Gazebo Harmonic, clone `tutorial/start`, build | colcon, workspaces | Empty workspace builds |
| 1 | Joint model and a first node that publishes a pose | Joint limits, clamping | `ros2 topic echo /braccio/joint_command` |
| 2 | Write the URDF step by step: links, joints, meshes | Frames, `rpy`, joint axes | Arm in RViz, moved with sliders |
| 3 | Convert servo degrees to URDF radians | Offsets, linear gripper mapping | Test passes |
| 4 | Forward kinematics | Rotation matrices, axis-angle, chaining transforms | Your FK matches RViz TF (`tf2_echo`) |
| 5 | Gazebo world, robot spawn, `ros2_control`, `/clock` bridge | Physics step, controllers | Arm standing in Gazebo |
| 6 | Trajectory bridge, then `pose_demo` | Move time scaled by move size | Arm waves |
| 7 | Inverse kinematics | `atan2` for the base, law of cosines for the two-link arm, tool-pitch search, checked against FK | `ik_pose_demo` reaches x/y/z points |
| 8 | Workspace layout: cubes and bins, one source of truth | Reach envelope | Scene in Gazebo; layout test passes |
| 9 | Overhead and gripper cameras, image bridge | Pinhole model, `fx` from field of view, pixel to table point | Both camera feeds in RViz |
| 10 | Colour detection | HSV thresholds, blob size filter | Cubes boxed by colour |
| 11 | Edge Impulse detection | Swappable model backend; confirmation window that ignores one-frame noise | Live detection boxes |
| 12 | Gripper-camera check | Area fraction of the image | Pick target confirmed |
| 13 | Grasping in sim (DetachableJoint), and why friction alone failed | Why physical grasping is hard in Gazebo | Cube carried without slipping |
| 14 | Pick-and-place state machine, in three passes: one hard-coded cube, then camera-driven, then all cubes with a final check | States and transitions | Full sort |
| 15 | Real hardware: Arduino UNO firmware and serial bridge | Serial protocol, non-blocking motion | Real arm moves |

Manual control, the web app and App Lab become optional appendices, not part
of the main path.

### Images to capture

Real screenshots and recordings only, no AI images. Diagrams marked (diagram)
are drawn by hand, e.g. in draw.io or Excalidraw. Store them in
`tutorial/images/`.

| Chapter | Images |
|---|---|
| Cover | `tutorial_cover_simulation.png`, `final_result_simulation.gif` |
| 0 | `ch00_workspace_build_success.png`, `ch00_skeleton_tree.png` |
| 1 | `ch01_topic_echo_joint_command.png` |
| 2 | `ch02_rviz_robot_model.png`, `ch02_joint_sliders.gif`, `ch02_braccio_frames_diagram.png` (diagram) |
| 3 | `ch03_servo_vs_urdf_angles_diagram.png` (diagram) |
| 4 | `ch04_tf_frames_rviz.png`, `ch04_tf2_echo_terminal.png` |
| 5 | `ch05_gazebo_arm_spawned.png`, `ch05_list_controllers_terminal.png` |
| 6 | `ch06_pose_demo_wave.gif` |
| 7 | `ch07_ik_two_link_diagram.png` (diagram), `ch07_ik_tool_pitch_diagram.png` (diagram), `ch07_ik_pose_demo_reach.gif` |
| 8 | `ch08_gazebo_workspace_cubes_bins.png`, `ch08_workspace_topdown_diagram.png` (diagram) |
| 9 | `ch09_overhead_camera_raw.png`, `ch09_gripper_camera_view.png`, `ch09_pinhole_projection_diagram.png` (diagram) |
| 10 | `ch10_hsv_mask_red.png`, `ch10_color_blob_detections.png` |
| 11 | `ch11_edge_impulse_dataset.png`, `ch11_edge_impulse_training_result.png`, `ch11_rviz_detection_boxes.png` |
| 12 | `ch12_gripper_verify_view.png` |
| 13 | `ch13_cube_slipping_before.gif`, `ch13_cube_attached_lift.gif` |
| 14 | `ch14_state_machine_diagram.png` (diagram), `ch14_single_cube_pick.gif`, `ch14_full_sort_sim.gif`, `ch14_rviz_task_state.png` |
| 15 | `ch15_uno_braccio_wiring.jpg`, `ch15_real_arm_pick.gif` |

---

## 2. New simulation features

In order of value. The first one makes the rest easier.

1. **Pick-and-place action server.** Replace the single `pick_place_demo`
   script with a ROS 2 action (e.g. `pick_place`, goal
   `{colour, target: bin | stack | position}`) that every other feature and
   the web UI can reuse.
2. **Random cube placement each run, plus grasps that handle rotated cubes.**
   Reset cube positions through Gazebo's set-pose service. Detect each cube's
   rotation with OpenCV `minAreaRect` and pass it to the existing
   `grasp_wrist_rotation`, which already accepts an object yaw.
3. **Stacking.** Build a tower. Release height becomes
   `n x cube size + margin` and placement must be precise. The overhead camera
   only sees the top cube, so checking the stack needs the cube's apparent
   size (closer looks bigger) or the gripper camera. A good tutorial chapter.
4. **Patterns.** A line, or a 3-2-1 pyramid. Mostly reuses stacking.
5. **Task buttons on the web dashboard.** "Sort all", "Stack red on blue",
   working in sim and on the real arm.
6. **Spawn cubes on demand** with `ros_gz_sim create`.
7. **Later:** two arms with a hand-off (see
   [two_arm_braccio_system_architecture.md](two_arm_braccio_system_architecture.md)),
   and a conveyor belt with moving cubes.

### Real-arm follow-ups

- Camera: measured height / position in `real_camera.yaml`, zoom measured from
  a cube ([camera.md](camera.md)). If a tilted camera turns out to be needed,
  `table_projection.HomographyProjection` (fit from 4+ known table points) is
  already written and tested.
- **Per-joint servo offsets** in a YAML file, and `GRIPPER_CLOSED` tuned for
  real cubes (there is no grasp assist on hardware).
