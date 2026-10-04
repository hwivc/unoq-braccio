import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from unoq_braccio_driver.braccio_workspace import CAMERA_XYZ, CUBES, grasp_topic


def generate_launch_description():
    share = get_package_share_directory("unoq_braccio_sim")
    xacro_path = os.path.join(share, "urdf", "braccio.urdf.xacro")
    world_path = os.path.join(share, "worlds", "workspace.world")
    controllers_path = os.path.join(share, "config", "controllers.yaml")
    mesh_dir = os.path.join(share, "meshes", "braccio_stedden")
    robot_description = {
        # value_type=str: otherwise launch_ros parses the URDF as YAML and
        # fails on any ": " in an XML comment.
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    xacro_path,
                    " controllers_file:=",
                    controllers_path,
                    " mesh_dir:=",
                    mesh_dir,
                ]
            ),
            value_type=str,
        ),
        "use_sim_time": True,
    }
    sim_time = {"use_sim_time": True}

    fallback_sim = DeclareLaunchArgument(
        "fallback_sim", default_value="false",
        description="Also run joint_state_simulator. Only for debugging without "
        "controllers: it publishes /joint_states and fights the broadcaster.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Open RViz with the robot, cameras and workspace markers.",
    )
    detector = DeclareLaunchArgument(
        "detector", default_value="true",
        description="Start sim_cube_detector on the overhead camera.",
    )
    detector_backend = DeclareLaunchArgument(
        "detector_backend", default_value="edge_impulse",
        description="Cube-finding backend: 'edge_impulse' (default) or 'color_blob'. "
        "See unoq_braccio_driver/cube_model_backend.py.",
    )
    model_path = DeclareLaunchArgument(
        "model_path", default_value="",
        description="Path to a .lite/.tflite model. Empty searches the repository "
        "for one (preferring a float32 model), then EDGE_IMPULSE_CUBE_MODEL.",
    )
    model_conf = DeclareLaunchArgument(
        "model_conf", default_value="0.3",
        description="Minimum score for a model detection to count as a cube.",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r ", world_path]}.items(),
    )

    # Gazebo -> ROS: sim clock and both camera streams. Without /clock every
    # use_sim_time node (controllers included) stays frozen at t=0.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/vision/overhead/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/vision/overhead/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/vision/gripper/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/vision/gripper/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ]
        # Grasp assist (see sim_grasp_attacher): attach/detach requests go
        # ROS -> Gazebo, each cube's DetachableJoint state comes back.
        + [
            arg
            for color in CUBES
            for arg in (
                f"{grasp_topic(color, 'attach')}@std_msgs/msg/Empty]gz.msgs.Empty",
                f"{grasp_topic(color, 'detach')}@std_msgs/msg/Empty]gz.msgs.Empty",
                f"{grasp_topic(color, 'state')}@std_msgs/msg/String[gz.msgs.StringMsg",
            )
        ],
        parameters=[sim_time],
        output="screen",
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "unoq_braccio", "-topic", "robot_description",
                   "-x", "0", "-y", "0", "-z", "0"],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        parameters=[sim_time],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        parameters=[sim_time],
        output="screen",
    )

    trajectory_bridge = Node(
        package="unoq_braccio_driver",
        executable="joint_trajectory_bridge",
        name="unoq_braccio_joint_trajectory_bridge",
        parameters=[sim_time],
        output="screen",
    )
    grasp_attacher = Node(
        package="unoq_braccio_driver",
        executable="sim_grasp_attacher",
        name="sim_grasp_attacher",
        parameters=[sim_time],
        output="screen",
    )
    joint_state_simulator = Node(
        package="unoq_braccio_driver",
        executable="joint_state_simulator",
        name="unoq_braccio_joint_state_simulator",
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration("fallback_sim")),
        output="screen",
    )
    cube_detector = Node(
        package="unoq_braccio_driver",
        executable="sim_cube_detector",
        name="sim_cube_detector",
        parameters=[sim_time, {
            "detector_backend": LaunchConfiguration("detector_backend"),
            "model_path": LaunchConfiguration("model_path"),
            "model_conf": ParameterValue(LaunchConfiguration("model_conf"), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration("detector")),
        output="screen",
    )

    gripper_detector = Node(
        package="unoq_braccio_driver",
        executable="sim_gripper_detector",
        name="sim_gripper_detector",
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration("detector")),
        output="screen",
    )
    markers = Node(
        package="unoq_braccio_driver",
        executable="workspace_markers",
        name="workspace_markers",
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )
    # The overhead camera is fixed in the world; publish it so RViz can show it.
    cam_x, cam_y, cam_z = CAMERA_XYZ
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", str(cam_x), "--y", str(cam_y), "--z", str(cam_z),
                   "--roll", "0", "--pitch", "1.5708", "--yaw", "0",
                   "--frame-id", "world", "--child-frame-id", "overhead_camera"],
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(share, "rviz", "braccio.rviz")],
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            fallback_sim,
            rviz_arg,
            detector,
            detector_backend,
            model_path,
            model_conf,
            gz_sim,
            bridge,
            state_publisher,
            spawn,
            # Controllers are started in order, each once the previous one exits,
            # instead of on a fixed timer that races Gazebo start-up.
            RegisterEventHandler(
                OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=joint_state_broadcaster, on_exit=[arm_controller])
            ),
            trajectory_bridge,
            grasp_attacher,
            joint_state_simulator,
            cube_detector,
            gripper_detector,
            markers,
            camera_tf,
            rviz,
        ]
    )
