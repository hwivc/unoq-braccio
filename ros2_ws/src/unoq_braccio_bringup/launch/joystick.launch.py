from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_id = LaunchConfiguration("device_id")
    deadzone = LaunchConfiguration("deadzone")

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_id", default_value="0", description="Joystick device ID (/dev/input/jsX)"),
            DeclareLaunchArgument("deadzone", default_value="0.12", description="Joystick deadzone threshold"),

            # Standard ROS 2 Joy Node
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[
                    {
                        "device_id": device_id,
                        "deadzone": deadzone,
                        "autorepeat_rate": 20.0,
                    }
                ],
                output="screen",
            ),

            # Braccio Teleoperation & Gripper Calibration Node
            Node(
                package="unoq_braccio_driver",
                executable="joystick_teleop",
                name="unoq_braccio_joystick_teleop",
                parameters=[
                    {
                        "deadzone": deadzone,
                        "command_topic": "/braccio/joint_command",
                    }
                ],
                output="screen",
            ),
        ]
    )
