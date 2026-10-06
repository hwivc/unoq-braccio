"""Camera-driven pick and place on the real arm. Start real.launch.py first.

    ros2 launch unoq_braccio_bringup real_pick_place.launch.py
    ros2 launch unoq_braccio_bringup real_pick_place.launch.py gripper_camera:=true

Use the same gripper_camera, camera_config and workspace_config values as
real.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from unoq_braccio_bringup.real_camera import read_camera_config


def generate_launch_description():
    arg = LaunchConfiguration

    def pick_place(context):
        camera = read_camera_config(arg("camera_config").perform(context))
        return [Node(
            package="unoq_braccio_driver",
            executable="pick_place_demo",
            name="pick_place_demo",
            parameters=[{
                "use_gripper_camera": ParameterValue(arg("gripper_camera"), value_type=bool),
                "workspace_config": arg("workspace_config"),
                "cube_size": camera["cube_size"],
                "step_wait": ParameterValue(arg("step_wait"), value_type=float),
            }],
            output="screen",
        )]

    return LaunchDescription([
        DeclareLaunchArgument("gripper_camera", default_value="false",
                              description="Check the cube with the gripper camera before and after grasping."),
        DeclareLaunchArgument("camera_config",
                              default_value="~/.ros/braccio_setup.yaml"),
        DeclareLaunchArgument("workspace_config",
                              default_value="~/.ros/braccio_setup.yaml"),
        DeclareLaunchArgument("step_wait", default_value="3.0",
                              description="Seconds to wait for each arm move to finish."),
        OpaqueFunction(function=pick_place),
    ])
