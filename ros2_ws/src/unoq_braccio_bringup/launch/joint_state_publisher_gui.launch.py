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
    rviz_config = os.path.join(sim_share, "rviz", "braccio.rviz")

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch a separate RViz window (set true if sim.launch.py is not running with RViz)",
    )
    forward_arg = DeclareLaunchArgument(
        "forward_to_arm",
        default_value="true",
        description="Forward GUI slider movements directly to /braccio/joint_command to move Gazebo or physical arm",
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    xacro_path,
                    " mesh_dir:=",
                    mesh_dir,
                    # left_gripper mimics gripper, so one slider drives both fingers.
                    " gui_mimic:=true",
                ]
            ),
            value_type=str,
        )
    }

    # GUI slider window for manual joint positioning
    gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[robot_description],
        remappings=[
            ("/joint_states", "/gui/joint_states"),
        ],
        output="screen",
    )

    # Bridge node: converts GUI radian sliders into Braccio degrees and drives the arm
    bridge_node = Node(
        package="unoq_braccio_driver",
        executable="gui_to_joint_command",
        name="gui_to_joint_command",
        parameters=[
            {
                "input_topic": "/gui/joint_states",
                "output_topic": "/braccio/joint_command",
            }
        ],
        condition=IfCondition(LaunchConfiguration("forward_to_arm")),
        output="screen",
    )

    # Optional RViz for standalone visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_gui",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_arg,
            forward_arg,
            gui_node,
            bridge_node,
            rviz_node,
        ]
    )
