"""Real arm with cameras: Arduino UNO over USB + overhead camera (+ optional gripper camera).

    ros2 launch unoq_braccio_bringup real.launch.py camera:=http://192.168.1.192:8080/video
    ros2 launch unoq_braccio_bringup real.launch.py camera:=0 serial_port:=/dev/braccio
    ros2 launch unoq_braccio_bringup real.launch.py camera:=0 gripper_camera:=true gripper_camera_source:=1

``camera`` / ``gripper_camera_source``: a USB index ("0"), a device path, or a
stream URL (Android "IP Webcam": http://<phone-ip>:8080/video).

Starts: the serial bridge to the UNO (always USB), the overhead camera, the
cube detector, and RViz. The camera's measured height and position, and the
cube size, come from config/real_camera.yaml (docs/camera.md). Then run the
task with real_pick_place.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from unoq_braccio_bringup.real_camera import read_camera_config


def generate_launch_description():
    share = get_package_share_directory("unoq_braccio_bringup")
    arg = LaunchConfiguration

    args = [
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0",
                              description="Arduino UNO USB serial port (/dev/braccio after setup)."),
        DeclareLaunchArgument("speed", default_value="60",
                              description="Arm speed, degrees per second of the fastest joint."),
        DeclareLaunchArgument("camera", default_value="0",
                              description="Overhead camera: USB index, /dev/videoN, or stream URL."),
        DeclareLaunchArgument("camera_width", default_value="640",
                              description="Resize camera frames to this width (0 = as is)."),
        DeclareLaunchArgument("gripper_camera", default_value="false",
                              description="Use a camera on the gripper for pick checks."),
        DeclareLaunchArgument("gripper_camera_source", default_value="1",
                              description="Gripper camera: USB index, /dev/videoN, or stream URL."),
        DeclareLaunchArgument("camera_config",
                              default_value="~/.ros/braccio_setup.yaml",
                              description="Written by real_setup: camera, cube size, colours, drop points."),
        DeclareLaunchArgument("workspace_config",
                              default_value="~/.ros/braccio_setup.yaml",
                              description="Cube colours and drop points (the same real_setup file)."),
        DeclareLaunchArgument("detector_backend", default_value="edge_impulse",
                              description="'edge_impulse' or 'color_blob'."),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("model_conf", default_value="0.3"),
        DeclareLaunchArgument("rviz", default_value="true"),
    ]

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "hardware.launch.py")),
        launch_arguments={
            "serial_port": arg("serial_port"),
            "speed": arg("speed"),
            "rviz": arg("rviz"),
        }.items(),
    )

    overhead_camera = Node(
        package="unoq_braccio_driver",
        executable="camera_node",
        name="overhead_camera",
        parameters=[{
            "camera": arg("camera"),
            "topic": "/vision/overhead/image_raw",
            "frame_id": "overhead_camera",
            "width": ParameterValue(arg("camera_width"), value_type=int),
        }],
        output="screen",
    )

    def cube_detector(context):
        try:
            camera = read_camera_config(arg("camera_config").perform(context))
        except RuntimeError as exc:
            # First run: start arm + camera without detection so real_setup can run.
            return [LogInfo(msg=f"{exc}. Starting without cube detection.")]
        return [Node(
            package="unoq_braccio_driver",
            executable="sim_cube_detector",
            name="cube_detector",
            parameters=[{
                "use_measured_camera": True,
                **camera,
                "workspace_config": arg("workspace_config"),
                "detect_bins": False,
                "detector_backend": arg("detector_backend"),
                "model_path": arg("model_path"),
                "model_conf": ParameterValue(arg("model_conf"), value_type=float),
            }],
            output="screen",
        )]

    gripper_on = IfCondition(arg("gripper_camera"))
    gripper_camera = Node(
        package="unoq_braccio_driver",
        executable="camera_node",
        name="gripper_camera",
        parameters=[{
            "camera": arg("gripper_camera_source"),
            "topic": "/vision/gripper/image_raw",
            "frame_id": "gripper_camera",
            "width": 320,
        }],
        condition=gripper_on,
        output="screen",
    )
    gripper_detector = Node(
        package="unoq_braccio_driver",
        executable="sim_gripper_detector",
        name="gripper_detector",
        parameters=[{"workspace_config": arg("workspace_config")}],
        condition=gripper_on,
        output="screen",
    )

    markers = Node(
        package="unoq_braccio_driver",
        executable="workspace_markers",
        name="workspace_markers",
        parameters=[{"workspace_config": arg("workspace_config"), "show_camera": False}],
        condition=IfCondition(arg("rviz")),
        output="screen",
    )

    return LaunchDescription(
        args + [hardware, overhead_camera, OpaqueFunction(function=cube_detector),
                gripper_camera, gripper_detector, markers]
    )
