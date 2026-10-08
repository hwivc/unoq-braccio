from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_launch = PathJoinSubstitution(
        [FindPackageShare("unoq_braccio_sim"), "launch", "gazebo.launch.py"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("fallback_sim", default_value="false"),
            DeclareLaunchArgument("detector", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("detector_backend", default_value="edge_impulse"),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("model_conf", default_value="0.3"),
            DeclareLaunchArgument("web", default_value="false",
                                  description="Also start the web dashboard (port web_port)."),
            DeclareLaunchArgument("web_port", default_value="8000"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "fallback_sim": LaunchConfiguration("fallback_sim"),
                    "detector": LaunchConfiguration("detector"),
                    "rviz": LaunchConfiguration("rviz"),
                    "detector_backend": LaunchConfiguration("detector_backend"),
                    "model_path": LaunchConfiguration("model_path"),
                    "model_conf": LaunchConfiguration("model_conf"),
                }.items(),
            ),
            Node(
                package="unoq_braccio_driver",
                executable="web_dashboard",
                name="web_dashboard",
                parameters=[{"port": ParameterValue(LaunchConfiguration("web_port"), value_type=int),
                             "mode": "sim"}],
                condition=IfCondition(LaunchConfiguration("web")),
                output="screen",
            ),
        ]
    )
