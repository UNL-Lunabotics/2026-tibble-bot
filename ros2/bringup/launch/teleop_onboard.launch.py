from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    control_pkg = FindPackageShare("control")
    description_pkg = FindPackageShare("description")
    realsense_pkg = FindPackageShare("realsense2_camera")

    robot_description_content = ParameterValue(
        Command(
            [
                "xacro ",
                PathSubstitution(description_pkg) / "urdf" / "tibble.urdf.xacro",
                " use_sim:=false",
                " use_control:=true"
            ]
        ),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # --- Nodes ---
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathSubstitution(control_pkg) / "config" / "tibble_controller.yaml"
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    tibble_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tibble_controller", "--controller-manager", "/controller_manager"],
    )

    delay_tibble_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[tibble_controller_spawner],
        )
    )

    camera_front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_pkg, '/launch/rs_launch.py']),
        launch_arguments={
            'camera_name': 'camera_front',
            'serial_no': "'033322071026'",
            'enable_depth': 'true',
            'enable_color': 'true',
        }.items()
    )

    # camera_rear = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([realsense_pkg, '/launch/rs_launch.py']),
    #     launch_arguments={
    #         'camera_name': 'camera_rear',
    #         'serial_no': '_INSERT_SERIAL_NUMBER_2_',    # TODO
    #         'enable_depth': 'true',
    #         'enable_color': 'true',
    #     }.items()
    # )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_tibble_controller_spawner,
        camera_front,
        # camera_rear,
        foxglove_bridge
    ])