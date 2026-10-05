"""Camera-driven pick and place on the real arm. Start real.launch.py first.

    ros2 launch unoq_braccio_bringup real_pick_place.launch.py
    ros2 launch unoq_braccio_bringup real_pick_place.launch.py gripper_camera:=true

Use the same gripper_camera and workspace_config values as real.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("unoq_braccio_bringup")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("gripper_camera", default_value="false",
                              description="Check the cube with the gripper camera before and after grasping."),
        DeclareLaunchArgument("workspace_config",
                              default_value=os.path.join(share, "config", "real_workspace.yaml")),
        DeclareLaunchArgument("step_wait", default_value="3.0",
                              description="Seconds to wait for each arm move to finish."),
        Node(
            package="unoq_braccio_driver",
            executable="pick_place_demo",
            name="pick_place_demo",
            parameters=[{
                "use_gripper_camera": ParameterValue(arg("gripper_camera"), value_type=bool),
                "workspace_config": arg("workspace_config"),
                "step_wait": ParameterValue(arg("step_wait"), value_type=float),
            }],
            output="screen",
        ),
    ])
