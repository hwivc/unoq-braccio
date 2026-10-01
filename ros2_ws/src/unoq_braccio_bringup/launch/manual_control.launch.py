from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    input_type = LaunchConfiguration("input_type")
    controller_type = LaunchConfiguration("controller_type")
    device_id = LaunchConfiguration("device_id")
    deadzone = LaunchConfiguration("deadzone")

    is_joystick = PythonExpression(["'", input_type, "'.lower() == 'joystick'"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_type",
                default_value="joystick",
                description="Control input mode: 'joystick' or 'keyboard'",
            ),
            DeclareLaunchArgument(
                "controller_type",
                default_value="ucom",
                description="Joystick button mapping profile: 'ucom' or 'xbox'",
            ),
            DeclareLaunchArgument(
                "device_id",
                default_value="0",
                description="Joystick device index (/dev/input/jsX)",
            ),
            DeclareLaunchArgument(
                "deadzone",
                default_value="0.12",
                description="Stick deadzone threshold (0.0 to 1.0)",
            ),

            # ROS 2 standard joy_node (only started when input_type == 'joystick')
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
                condition=IfCondition(is_joystick),
                output="screen",
            ),

            # Braccio Manual Control & Calibration Node
            Node(
                package="unoq_braccio_driver",
                executable="manual_control",
                name="manual_control",
                parameters=[
                    {
                        "input_type": input_type,
                        "controller_type": controller_type,
                        "deadzone": deadzone,
                        "command_topic": "/braccio/joint_command",
                    }
                ],
                prefix="xterm -e" if False else "",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
