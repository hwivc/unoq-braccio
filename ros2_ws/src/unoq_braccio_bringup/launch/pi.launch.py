"""Raspberry Pi (Ubuntu Server, no screen): the real arm, the camera and the web dashboard.

    ros2 launch unoq_braccio_bringup pi.launch.py
    ros2 launch unoq_braccio_bringup pi.launch.py camera:=grove:/dev/grove_vision
    ros2 launch unoq_braccio_bringup pi.launch.py camera:=0 web_port:=8080

real.launch.py with headless defaults: no RViz, the web dashboard on, and
both devices found by USB id on every (re)connect, whichever port they land
on and whatever order they are plugged in: the UNO (``serial_port:=auto``)
and the Grove Vision AI V2 as the overhead camera (``camera:=grove``).
When auto-detection cannot decide, or for another camera, pick one in the
web page's Settings > Devices; no restart needed. Everything is watched and
driven from a browser: http://<pi-address>:8000 (printed at start-up).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("unoq_braccio_bringup")
    arg = LaunchConfiguration

    args = [
        DeclareLaunchArgument("serial_port", default_value="auto",
                              description="Arduino UNO serial port, or auto to find it by USB id."),
        DeclareLaunchArgument("camera", default_value="grove",
                              description="Overhead camera: grove[:<port>], USB index, /dev/videoN, or stream URL."),
        DeclareLaunchArgument("grove_resolution", default_value="2",
                              description="Grove Vision AI V2: 0 = 240x240, 1 = 480x480, 2 = 640x480."),
        DeclareLaunchArgument("speed", default_value="60",
                              description="Arm speed, degrees per second of the fastest joint."),
        DeclareLaunchArgument("web_port", default_value="8000",
                              description="Web dashboard port."),
    ]

    real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "real.launch.py")),
        launch_arguments={
            "serial_port": arg("serial_port"),
            "camera": arg("camera"),
            "grove_resolution": arg("grove_resolution"),
            "speed": arg("speed"),
            "web_port": arg("web_port"),
            "rviz": "false",
            "web": "true",
        }.items(),
    )

    return LaunchDescription(args + [real])
