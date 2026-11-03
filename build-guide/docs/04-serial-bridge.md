# 4. A ROS node owns the arm

## Objective

A node, `serial_bridge`, that subscribes to `/braccio/joint_command`, sends `M`
lines to the UNO, polls `S`, and publishes `/braccio/firmware_status` and
`/joint_states`. It reconnects by itself when the cable or the board drops.

## Why we need this now

Only one program can own a serial port. Turning it into a ROS node means every
other node, now and later, moves the arm the same simple way: **publish a
`JointState`**. None of them needs to know about USB.

## Create

1. `serial_bridge.py` in the driver package.
2. Copy `usb_devices.py` as-is from the original (support code). It finds the UNO
   by its USB id, so the port name doesn't matter, and remembers the dashboard's
   device choices.
3. Add the entry point: `"serial_bridge = unoq_braccio_driver.serial_bridge:main",`
4. Chapter 5 adds `braccio_kinematics.py`. Until then, copy it as-is, because the
   bridge imports one function from it.

## The code

### Set-up: parameters, topics, a timer and a reader thread

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py SerialBridge.__init__

Two loops share the port:

- A **timer** (`tick`, every 20 ms) does all the *writing*.
- A **thread** (`read_loop`) blocks on `readline()` and does all the *reading*.

They share `self.serial` under `self.lock`.

### Receiving commands

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py SerialBridge.on_command

This doesn't write to the port. It just records the wish and sets `dirty`. Many
commands can arrive between ticks (the dashboard sliders send a lot); only the
newest gets sent, which keeps the UNO's small input buffer from overflowing.

### The heartbeat

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py SerialBridge.tick

Read it as a little state machine:

1. **No port** → try to open it.
2. **Open but not ready** → probe with `S`.
3. **Ready** → send the newest command, and poll status.

### Opening and losing the port

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py SerialBridge.open_port

- `exclusive=True` stops a second program from silently sharing the port.
- The **DTR pulse** restarts the UNO, so every connection starts from the known start pose.

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py SerialBridge.close_port

### Writing and reading lines

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py SerialBridge.write

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py SerialBridge.read_loop

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py SerialBridge.on_line

The first `STAT` sets `baseline`, the arm's real position. Commands are merged
over it, so a command naming only the gripper keeps the other joints where they are.

### The rest

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py SerialBridge.forget_commands

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py SerialBridge.mark_ready

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py SerialBridge.publish_joint_states

@code ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/serial_bridge.py main

Also type the remaining small methods (`wanted`, `resolve_port`, `check_choice`,
`publish_device_status`, `load_rviz_mirror`, `destroy_node`) and the imports
(lines 1–58) from the original file. They connect the bridge to `usb_devices`.

## Run it

```bash
cd $NEW/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 run unoq_braccio_driver serial_bridge --ros-args -p serial_port:=auto
# second terminal:
ros2 topic echo /braccio/firmware_status
ros2 topic pub --once /braccio/joint_command sensor_msgs/msg/JointState \
  "{name: [gripper], position: [73]}"
```

Passes when `STAT` lines stream and the gripper opens. Unplug and replug the USB
cable: it should reconnect by itself.

## What's next

ROS can move the arm, but nothing *sees* it: `/joint_states` are just numbers.
Next we describe the arm's shape so RViz and the web page can draw it, and write
our first launch file: chapter 5.
