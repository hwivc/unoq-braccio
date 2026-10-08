"""Pick and place with a model taught by teach_pick. Start real.launch.py first.

    ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=desk
    ros2 launch unoq_braccio_bringup learned_pick_place.launch.py session:=desk model:=model_0010

Teach first:  ros2 run unoq_braccio_driver teach_pick --ros-args -p session:=desk
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("session", default_value="default",
                              description="Folder in ~/.ros/braccio_teach/ that teach_pick wrote."),
        DeclareLaunchArgument("model", default_value="",
                              description="'' = the latest model, or e.g. model_0010."),
        DeclareLaunchArgument("setup_file", default_value="~/.ros/braccio_setup.yaml",
                              description="Cube colours and gripper values (from real_setup)."),
        DeclareLaunchArgument("unsure_px", default_value="60.0",
                              description="Leave cubes further than this from every example."),
        Node(
            package="unoq_braccio_driver",
            executable="learned_pick_demo",
            name="learned_pick_demo",
            parameters=[{
                "session": arg("session"),
                "model": ParameterValue(arg("model"), value_type=str),
                "setup_file": arg("setup_file"),
                "unsure_px": ParameterValue(arg("unsure_px"), value_type=float),
            }],
            output="screen",
        ),
    ])
