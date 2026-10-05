"""Real arm: Arduino UNO + Braccio shield over USB serial.

    ros2 launch unoq_braccio_bringup hardware.launch.py
    ros2 launch unoq_braccio_bringup hardware.launch.py serial_port:=/dev/braccio speed:=40 rviz:=true

The serial bridge turns /braccio/joint_command into firmware commands and
publishes the arm's real position on /joint_states; robot_state_publisher
turns that into TF so RViz shows the real arm.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory("unoq_braccio_sim")
    xacro_path = os.path.join(sim_share, "urdf", "braccio.urdf.xacro")
    mesh_dir = os.path.join(sim_share, "meshes", "braccio_stedden")
    robot_description = ParameterValue(
        Command(["xacro ", xacro_path, " mesh_dir:=", mesh_dir]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port", default_value="/dev/ttyACM0",
                description="UNO serial port; /dev/braccio after scripts/setup_uno_serial.sh.",
            ),
            DeclareLaunchArgument(
                "speed", default_value="60",
                description="Speed of the fastest-moving joint, degrees per second (10-180).",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="false", description="Open RViz showing the real arm."
            ),
            Node(
                package="unoq_braccio_driver",
                executable="serial_bridge",
                name="braccio_serial_bridge",
                parameters=[{
                    "serial_port": LaunchConfiguration("serial_port"),
                    "speed_deg_s": ParameterValue(LaunchConfiguration("speed"), value_type=float),
                }],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(sim_share, "rviz", "braccio.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
