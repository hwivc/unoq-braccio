from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    input_type = LaunchConfiguration("input_type")
    controller_type = LaunchConfiguration("controller_type")
    device_id = LaunchConfiguration("device_id")
    device_name = LaunchConfiguration("device_name")
    deadzone = LaunchConfiguration("deadzone")

    is_joystick = PythonExpression(["'", input_type, "'.lower() == 'joystick'"])

    # Convert device_id safely to integer for C++ joy_node parameter
    device_id_int = PythonExpression([
        "int('", device_id, "'.split('js')[-1]) if 'js' in '", device_id, "' else int('", device_id, "')"
    ])
    deadzone_float = PythonExpression(["float('", deadzone, "')"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_type",
                default_value="joystick",
                description="Control input mode: 'joystick' or 'keyboard' "
                "(on Linux, keyboard mode needs 'ros2 run' for an interactive terminal)",
            ),
            DeclareLaunchArgument(
                "controller_type",
                default_value="ucom",
                description="Joystick button mapping profile: 'ucom' or 'xbox'",
            ),
            DeclareLaunchArgument(
                "device_id",
                default_value="0",
                description="Joystick USB device index (e.g. 0 for /dev/input/js0, 1 for /dev/input/js1, or full /dev/input/jsX)",
            ),
            DeclareLaunchArgument(
                "device_name",
                default_value="",
                description="Optional joystick device name string match",
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
                        "device_id": device_id_int,
                        "device_name": device_name,
                        # manual_control applies its own deadzone and response
                        # curve; a second large one here would stack with it.
                        "deadzone": 0.05,
                        "autorepeat_rate": 30.0,
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
                        "deadzone": deadzone_float,
                        "command_topic": "/braccio/joint_command",
                    }
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
